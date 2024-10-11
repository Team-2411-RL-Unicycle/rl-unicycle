import asyncio
import logging
import time
from multiprocessing import Queue
from typing import List, Union

# For importing data files from the source, independent of the installation method
import pkg_resources

from rluni.controller import (ControlInput, Controller, LQRController,
                              PIDController, RLController, TestController)
from rluni.fusion.AHRSfusion import AHRSfusion
from rluni.icm20948.imu_lib import ICM20948
from rluni.motors.MN6007 import MN6007
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

from . import teledata as td

# Create a logger
logger = logging.getLogger(__name__)


class RobotSystem:
    """
    The RobotSystem class manages the core functionalities of a robot, including initializing sensors,
    actuators, and handling the control loop for real-time operations.

    Attributes:
        LOOP_TIME (float): The fixed period for the control loop in seconds (e.g., 0.01 for 100Hz).
        WRITE_DUTY (float): The fraction of the loop period before actuators are updated
        MAX_TORQUE (float): The maximum torque to request from motors, used for testing and safety constraints.
        xmotor (MN6007 or None): The motor controller instance, if motors are started.
        itr (int): An iteration counter for the control loop.
    """

    def __init__(
        self,
        send_queue: Queue,
        receive_queue: Queue,
        start_motors=True,
        controller_type: str = "test",
        config_file=None,
    ):
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        # Load and parse the configuration file
        self._load_config(config_file)

        # Initialize IMU and sensor fusion
        self.imu = ICM20948(config_file=self.imu_config)
        self.sensor_fusion = AHRSfusion(
            sample_rate=int(1 / self.LOOP_TIME), config_file=self.imu_config
        )

        # Initialize motor controller (if enabled)
        self.xmotor = MN6007() if start_motors else None
        self.motors_enabled = start_motors

        # Initialize controller type based on argument
        self.controller_type = controller_type
        self.controller = self._get_controller(controller_type)

        self.itr = int(0)  # Cycle counter

    def _load_config(self, config_file):
        """
        Private method to load and parse the RWIP configuration file.
        """
        # Load the RWIP configuration file (use the default if none provided)
        if config_file is None:
            config_file = pkg_resources.resource_filename(
                "rluni.configs.rwip", "default.yaml"
            )
            logger.warning(
                f"No RWIP configuration file provided. Using default configuration: {config_file}"
            )

        config = load_config_file(config_file)

        # Validate and set configuration values for control parameters
        self.LOOP_TIME = gvcv(config, "RobotSystem.loop_time", float, required=True)
        self.WRITE_DUTY = gvcv(config, "RobotSystem.write_duty", float, required=True)
        self.MAX_TORQUE = gvcv(config, "RobotSystem.max_torque", float, required=True)
        self.sensor_calibration_delay = gvcv(
            config, "RobotSystem.calibration_delay", float, required=True
        )

        # Validate and resolve paths for IMU config and RL model
        self.imu_config = gvcv(config, "RobotSystem.imu_config", str, required=True)
        self.imu_config = pkg_resources.resource_filename("rluni", self.imu_config)

        self.rlmodel_path = gvcv(
            config, "RobotSystem.rlmodel_path", str, required=False
        )
        self.rlmodel_path = pkg_resources.resource_filename("rluni", self.rlmodel_path)

        self.pid_config_path = gvcv(
            config, "RobotSystem.pid_config", str, required=False
        )
        self.pid_config_path = pkg_resources.resource_filename(
            "rluni", self.pid_config_path
        )

    def _get_controller(self, controller_type: str) -> Controller:
        """Initialize the controller based on the type ('pid', 'rl', 'lqr', 'test')."""
        if controller_type == "pid":
            return PIDController(config_file=self.pid_config_path)
        elif controller_type == "rl":
            return RLController(model_pth=self.rlmodel_path)
        elif controller_type == "lqr":
            return LQRController()
        elif controller_type == "test":
            return TestController()
        else:
            raise ValueError(f"Unsupported controller type: {controller_type}")

    async def start(self, shutdown_event):
        """
        Initializes actuators and starts the control loop.
        """
        # Init actuators
        if self.xmotor is not None:
            await self.xmotor.start()
        try:
            await self.control_loop(shutdown_event)
        except asyncio.CancelledError:
            logger.info("Robot Loop Shutdown Signal.")
            # Set the shutdown event to ensure control_loop exits
            shutdown_event.set()
            raise

    async def control_loop(self, shutdown_event):
        """
        The main control loop for the robot. Executes sensor reading, state estimation, control decision making,
        and actuator commands at a fixed rate defined by LOOP_TIME.
        """
        loop_period = self.LOOP_TIME
        torque_request = 0

        while not shutdown_event.is_set():
            # Start Loop Timer and increment loop iteration
            loop_start_time = time.time()
            self.itr += 1

            # Debugging Data Example (add more data as needed)
            tele_debug_data = td.DebugData()
            tele_debug_data.add_data(example_debug_data=42)

            # Sensor reading and fusion
            imudata = td.IMUData(*self.imu.read_accelerometer_gyro(convert=True))
            euler_angles = td.EulerAngles(
                *self.sensor_fusion.update(
                    imudata.get_gyro(), imudata.get_accel(), delta_time=loop_period
                )[0]
            )

            # Update robot state and parameters
            if self.xmotor is not None:
                await self.xmotor.update_state()

            # Control logic
            control_input = ControlInput(
                pendulum_angle=euler_angles.y,  # [degrees]
                pendulum_vel=imudata.gyro_z,  # [degrees / s] positive CCW
                wheel_vel=(
                    0 if self.xmotor is None else self.xmotor.state["VELOCITY"]
                ),  # [rev / s] positive CW
                roll_torque=torque_request,  # [N * m]
            )

            # Change to negative convention due to motor
            torque_request = self.controller.get_torque(
                control_input, self.MAX_TORQUE - 0.001
            )

            ## DELAY UNTIL FIXED POINT ##
            self.precise_delay_until(loop_start_time + loop_period * self.WRITE_DUTY)

            # Apply control decision to robot actuators
            # SET TORQUE
            isCalibrating = self.itr < self.sensor_calibration_delay / self.LOOP_TIME
            if self.motors_enabled and not isCalibrating:
                # Only set the torque if not in sensor fusion calibration mode
                await self.xmotor.set_torque(
                    torque=torque_request, max_torque=self.MAX_TORQUE
                )

            ### SEND COMMS ###
            # Send out all data downsampled to (optional lower) rate
            if (self.itr % 1) == 0:
                control_data = td.ControlData(
                    loop_time=loop_period, torque_request=float(torque_request)
                )
                data_list = [imudata, euler_angles, control_data, tele_debug_data]
                if self.xmotor is not None:
                    motor_state = td.MotorState.from_dict(self.xmotor.state)
                    data_list.append(motor_state)
                await self._send_telemetry(data_packet=data_list)

            ### RECEIVE COMMS ###
            # Check for and handle new commands sent in via MQTT
            await self._handle_commands()

            ### CLOSE LOOP DELAY ###
            end_time = loop_start_time + self.LOOP_TIME
            loop_delay = self.precise_delay_until(end_time)
            loop_period = time.time() - loop_start_time

    """ ####################### 
        Command Handling Methods
        #######################  """

    async def _send_telemetry(
        self, data_packet: Union[List[td.TelemetryData], td.TelemetryData]
    ) -> None:
        """
        Send telemetry data to the telemetry queue.

        Args:
            data_list (Union[List[TelemetryData], TelemetryData]): The telemetry data to send (list or singletons)
        """
        self.send_queue.put(data_packet)

    async def _handle_commands(self):
        """Handle commands from the receive queue."""
        while not self.receive_queue.empty():
            message = self.receive_queue.get()
            if isinstance(message, dict) and len(message) == 1:
                command, value = next(iter(message.items()))
                await self._execute_command(command, value)
            else:
                logger.error(f"Invalid command format: {message}")

    async def _execute_command(self, command: str, value):
        """Execute a specific command."""
        if command == "power":
            await self._handle_power_command(command, value)
        elif command in {"P", "I", "D"}:
            await self._handle_pid_command(command, value)
        elif command == "controller":
            await self._handle_controller_switch(command, value)
        else:
            logger.error(f"Unrecognized command: {command}")

    async def _handle_power_command(self, command: str, value: bool):
        """
        Handles power on/off commands for the robot.

        Args:
            value (bool): True to power on the robot, False to power off.
        """
        # Ensure the value is a boolean
        if isinstance(value, bool):
            if value:
                # Code to execute if the power command is True (e.g., turn on the robot)
                if self.xmotor is not None:
                    self.motors_enabled = True
                    logger.info(f"Motors power enabled for {str(self.xmotor)}.")
                else:
                    logger.info(f"No motor to turn on.")
            else:
                # Code to execute if the power command is False (e.g., turn off the robot)
                if self.xmotor is not None:
                    await self.xmotor.stop()
                    logger.info(f"Motor power disabled to {str(self.xmotor)}.")
                    self.motors_enabled = False
                else:
                    self.motors_enabled = False
                    logger.info("No motor to turn off.")
        else:
            # Log an error or handle the case where the value is not a boolean
            logger.error(f"Expected a boolean for the power command, but got: {value}")

    async def _handle_pid_command(self, command: str, value: float):
        if self.controller_type != "pid":
            logger.warning(
                f"WARNING: Received PID command, but controller type is: {self.controller_type}. Ignoring command."
            )
            return
        if not isinstance(value, float):
            logger.warning(
                f"WARNING: Expected a float for the pid command, but got: {value}. Ignoring command."
            )
            return

        self.controller.update_parameter(command, value)
        msg = f"Setting PID parameter {command} to value {value}."
        logger.info(msg)
        return

    async def _handle_controller_switch(self, command: str, value: str):
        """
        Handles controller switch commands for the robot.

        Args:
            value (str): The controller type to switch to.
        """
        # Ensure the value is a string
        if isinstance(value, str):
            try:
                # Delegate the controller initialization to the private method
                self.controller = self._get_controller(value)
                self.controller_type = value
                logger.info(f"Controller switched to: {value}")
            except ValueError as e:
                logger.error(str(e))
        else:
            # Log an error or handle the case where the value is not a string
            logger.error(
                f"Expected a string for the controller switch command, but got: {value}"
            )

    def precise_delay_until(self, end_time):
        """
        Sleep until the specified end time, then return the delay time (overshoot)
        """
        # Lower accuracy sleep loop
        while True:
            now = time.time()
            remaining = end_time - now
            if remaining <= 0.0007:
                break
            time.sleep(remaining / 2)

        # Busy-while loop to get accurate cutoff at end
        while True:
            delay = time.time() - end_time
            if delay >= 0:
                return delay

    """ Shutdown Methods """

    async def shutdown(self):
        """
        Shutdown the robot system and its components.
        """
        try:
            if self.xmotor is not None:
                await self.xmotor.shutdown()
                logger.info("Motors shutdown successfully.")
        except Exception as e:
            logger.exception(f"Error during RobotSystem shutdown: {e}")
