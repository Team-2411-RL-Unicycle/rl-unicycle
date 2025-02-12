import asyncio
import logging
import math
import time
from collections import namedtuple
from enum import Enum
from multiprocessing import Queue
from typing import Callable, List, Union, Tuple
from dataclasses import asdict
import moteus

import numpy as np

# For importing data files from the source, independent of the installation method
from importlib.resources import files

from rluni.controller.fullrobot import (
    ControlInput,
    Controller,
    LQRController,
    PIDController,
    RLController,
    MPCController,
    TestController,
)
from rluni.fusion.AHRSfusion import AHRSfusion
from rluni.icm20948.imu_lib import ICM20948
from rluni.motors.motors import MN2806, MN6007, Motor
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

from . import teledata as td

DEG_TO_RAD = math.pi / 180
REV_TO_RAD = 2 * math.pi

# Create a logger
logger = logging.getLogger(__name__)


class EnabledMotors(Enum):
    NONE = 0
    ROLL = 1
    YAW = 2
    PITCH = 3
    ROLL_PITCH = 4
    ALL = 5
    NUM_CONFIGS = 6


motors = namedtuple("motors", ["roll", "pitch", "yaw"])
torques = namedtuple("torques", ["roll", "pitch", "yaw"])


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
        motor_config=None,
    ):
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        # Load and parse the configuration file
        self._load_config(config_file)

        # Initialize IMU and sensor fusion
        self.imu1 = ICM20948(config_file=self.imu_config1)
        self.imu2 = ICM20948(config_file=self.imu_config2)
        self.sensor_fusion = AHRSfusion(
            sample_rate=int(1 / self.LOOP_TIME), config_file=self.imu_config1
        )

        self.motors: Tuple[Motor, ...] = None
        self._initialize_motors(motor_config)

        # Initialize controller type based on argument
        self.controller_type = controller_type
        self.controller = self._get_controller(controller_type)
        self.ema_control_input = None
        self.ema_alpha = 0.7  # .72 roll
        self.itr = int(0)  # Cycle counter

    def _initialize_motors(self, motor_config):
        # CHECK THIS
        self.transport = moteus.Fdcanusb(
            "/dev/serial/by-id/usb-mjbots_fdcanusb_5A70499D-if00"
        )
        # set self.motor_config using arg string
        if motor_config == "none" or None:
            self.motor_config = EnabledMotors.NONE
            self.motors = motors(None, None, None)
        elif motor_config == "roll":
            self.motors = motors(MN6007(4, "roll", self.transport), None, None)
            self.motor_config = EnabledMotors.ROLL
        elif motor_config == "yaw":
            self.motors = motors(None, None, MN2806(5, "yaw"), self.transport)
            self.motor_config = EnabledMotors.YAW
        elif motor_config == "pitch":
            self.motors = motors(None, MN6007(6, "pitch", self.transport), None)
            self.motor_config = EnabledMotors.PITCH
        elif motor_config == "roll_pitch" or motor_config == "pitch_roll":
            self.motors = motors(
                MN6007(4, "roll", self.transport),
                MN6007(6, "pitch", self.transport),
                None,
            )
            self.motor_config = EnabledMotors.ROLL_PITCH
        elif motor_config == "all":
            self.motors = motors(
                MN6007(4, "roll", self.transport),
                MN6007(6, "pitch", self.transport),
                MN2806(5, "yaw", self.transport),
            )
            self.motor_config = EnabledMotors.ALL

            self.enabled_motors = [motor for motor in self.motors if motor is not None]
        else:  # catch-all
            logger.warning("No valid motor configuration provided. Disabling motors.")
            self.motors = motors(None, None, None)
            self.motor_config = EnabledMotors.NONE
            self.enabled_motors = []

    def _load_config(self, config_file):
        """
        Private method to load and parse the robot configuration file.
        """
        # Load the robot configuration file (use the default if none provided)
        if config_file is None:
            config_file = "unicycle.yaml"
            logger.warning(
                f"No configuration file provided. Using default configuration: {config_file}"
            )

        config_file_path = files("rluni.configs.robot").joinpath(config_file)
        config_file = str(config_file_path)

        config = load_config_file(config_file)

        # Validate and set configuration values for control parameters
        self.LOOP_TIME = gvcv(config, "RobotSystem.loop_time", float, required=True)
        self.WRITE_DUTY = gvcv(config, "RobotSystem.write_duty", float, required=True)
        self.MAX_TORQUE_ROLL_PITCH = gvcv(
            config, "RobotSystem.max_torque_roll_pitch", float, required=True
        )
        self.MAX_TORQUE_YAW = gvcv(
            config, "RobotSystem.max_torque_yaw", float, required=True
        )
        self.sensor_calibration_delay = gvcv(
            config, "RobotSystem.calibration_delay", float, required=True
        )

        # Validate and resolve paths for IMU config and RL model
        self.imu_config1 = gvcv(config, "RobotSystem.imu_config1", str, required=True)
        self.imu_config1 = str(files("rluni").joinpath(self.imu_config1))
        self.imu_config2 = gvcv(config, "RobotSystem.imu_config2", str, required=True)
        self.imu_config2 = str(files("rluni").joinpath(self.imu_config2))

        self.rlmodel_path = gvcv(
            config, "RobotSystem.rlmodel_path", str, required=False
        )

        self.rlmodel_path = str(files("rluni").joinpath(self.rlmodel_path))

        self.pid_config_path = gvcv(
            config, "RobotSystem.pid_config", str, required=False
        )
        self.pid_config_path = str(files("rluni").joinpath(self.pid_config_path))

    def _get_controller(self, controller_type: str) -> Controller:
        """Initialize the controller based on the type ('pid', 'rl', 'lqr', 'test')."""
        if controller_type == "pid":
            return PIDController(config_file=self.pid_config_path)
        elif controller_type == "rl":
            return RLController(model_pth=self.rlmodel_path)
        elif controller_type == "lqr":
            return LQRController()
        elif controller_type == "mpc":
            return MPCController(dt=self.LOOP_TIME)
        elif controller_type == "test":
            return TestController()
        else:
            raise ValueError(f"Unsupported controller type: {controller_type}")

    async def start(self, shutdown_event):
        """
        Starts the control loop.
        """
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
        and actuator commands at a fixed rate defi                if self.motors.roll is not None:
                    await self.motors.roll.set_torque(torques.roll, self.MAX_TORQUE_ROLL_PITCH - 0.001)
                if self.motors.pitch is not None:
                    await self.motors.pitch.set_torque(torques.pitch, self.MAX_TORQUE_ROLL_PITCH - 0.001)
                if self.motors.yaw is not None:
                    await self.motors.yaw.set_torque(torques.yaw, self.MAX_TORQUE_YAW - 0.001)ned by LOOP_TIME.
        """
        loop_period = self.LOOP_TIME

        while not shutdown_event.is_set():
            # Start Loop Timer and increment loop iteration
            loop_start_time = time.time()
            timer_tele = td.LoopTimer()
            self.itr += 1

            # Debugging Data Example (add more data as needed)
            tele_debug_data = td.DebugData()
            tele_debug_data.add_data(example_debug_data=99)
            tele_debug_data.add_data(another_debug_data=42)

            # Sensor reading and fusion
            imudata1 = td.IMUData(1, *self.imu1.read_sensor_data(convert=True))
            imudata2 = td.IMUData(2, *self.imu2.read_sensor_data(convert=True))
            timer_tele.imu_read = time.time() - loop_start_time

            quaternion, internal_states, flags = self.sensor_fusion.update(
                imudata1.get_gyro(),
                imudata1.get_accel(),
                mag_data=imudata1.get_mag(),
                delta_time=loop_period,
            )
            rigid_body_state = td.EulerAngles(
                *self.sensor_fusion.euler_angles, *self.sensor_fusion.euler_rates
            )
            timer_tele.sensor_fusion = time.time() - loop_start_time

            # TODO: Try to get all 3 motor states in one call
            # Update robot state and parameters
            for motor in self.motors:
                if motor is not None:
                    await motor.update_state()
            timer_tele.motor_states = time.time() - loop_start_time

            control_input = ControlInput(
                euler_angle_roll_rads=rigid_body_state.x * DEG_TO_RAD,
                euler_angle_pitch_rads=rigid_body_state.y * DEG_TO_RAD,
                euler_angle_yaw_rads=rigid_body_state.z * DEG_TO_RAD,
                euler_rate_roll_rads_s=rigid_body_state.x_dot,
                euler_rate_pitch_rads_s=rigid_body_state.y_dot,
                euler_rate_yaw_rads_s=rigid_body_state.z_dot,
                motor_speeds_pitch_rads_s=(
                    0.0
                    if self.motors.pitch is None
                    else self.motors.pitch.state["VELOCITY"] * REV_TO_RAD
                ),
                motor_speeds_roll_rads_s=(
                    0.0
                    if self.motors.roll is None
                    else -self.motors.roll.state["VELOCITY"] * REV_TO_RAD
                ),
                motor_speeds_yaw_rads_s=(
                    0.0
                    if self.motors.yaw is None
                    else -self.motors.yaw.state["VELOCITY"] * REV_TO_RAD
                ),
            )

            if abs(control_input.motor_speeds_pitch_rads_s) > 10:
                logger.warning("Pitch motor speed too high, shutting down.")
                shutdown_event.set()

            self._update_ema_control_input(control_input)

            torques = self.controller.get_torques(
                self.ema_control_input, self.MAX_TORQUE_ROLL_PITCH - 0.001
            )
            timer_tele.control_decision = time.time() - loop_start_time

            ## DELAY UNTIL FIXED POINT ##
            self.precise_delay_until(loop_start_time + loop_period * self.WRITE_DUTY)
            timer_tele.duty_cycle_delay_time = time.time() - loop_start_time

            # Apply control decision to robot actuators
            # SET TORQUE
            # Only set the torque if not in sensor fusion calibration mode
            #  TODO: Can all motors be set in one transport or more efficiently?
            isCalibrating = self.itr < self.sensor_calibration_delay / self.LOOP_TIME


            # Control decision - but using transports
            if not isCalibrating and self.motor_config is not EnabledMotors.NONE:
                commands = []
                if self.motors.roll is not None:
                    commands.append(
                        self.motors.roll._c.make_position(
                            position=math.nan,
                            kp_scale=0.0,
                            kd_scale=0.0,
                            feedforward_torque=torques.roll,
                            maximum_torque=self.MAX_TORQUE_ROLL_PITCH - 0.001,
                        )
                    )
                if self.motors.pitch is not None:
                    commands.append(
                        self.motors.pitch._c.make_position(
                            position=math.nan,
                            kp_scale=0.0,
                            kd_scale=0.0,
                            feedforward_torque=torques.pitch,
                            maximum_torque=self.MAX_TORQUE_ROLL_PITCH - 0.001,
                        )
                    )
                if self.motors.yaw is not None:
                    commands.append(
                        self.motors.yaw._c.make_position(
                            position=math.nan,
                            kp_scale=0.0,
                            kd_scale=0.0,
                            feedforward_torque=torques.yaw,
                            maximum_torque=self.MAX_TORQUE_YAW - 0.001,
                        )
                    )

                await self.transport.cycle(commands)
                
            # Handle loop timer telemetry data processing
            timer_tele.torque_application = time.time() - loop_start_time
            timer_tele.convert_to_periods()  # convert clocked times to intervals (periods)
            timer_tele.end_loop_buffer = self.LOOP_TIME - (
                time.time() - loop_start_time
            )

            ### SEND COMMS ###
            # Send out all data downsampled to (optional lower) rate
            if (self.itr % 1) == 0:
                control_data = td.ControlData(
                    loop_time=loop_period,
                    torque_roll=float(torques.roll),
                    torque_pitch=float(torques.pitch),
                    torque_yaw=float(torques.yaw),
                )
                data_list = [
                    imudata1,
                    imudata2,
                    rigid_body_state,
                    control_data,
                    timer_tele,
                    tele_debug_data,
                ]
                for motor in self.motors:
                    if motor is not None:
                        data_list.append(
                            td.MotorState.from_dict(data=motor.state, name=motor.name)
                        )
                await self._send_telemetry(data_packet=data_list)

            ### RECEIVE COMMS ###
            # Check for and handle new commands sent in via MQTT
            await self._handle_commands()

            ### CLOSE LOOP DELAY ###
            end_time = loop_start_time + self.LOOP_TIME
            loop_delay = self.precise_delay_until(end_time)
            loop_period = time.time() - loop_start_time

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
            for motor in self.motors:
                if motor is not None:
                    await motor.shutdown()
            logger.info("Motors shut down successfully.")
        except Exception as e:
            logger.exception(f"Error during RobotSystem shutdown: {e}")

    def _update_ema_control_input(self, control_input):
        ema_fields = {
            # "euler_angle_roll_rads",
            # "euler_angle_pitch_rads",
            "euler_rate_roll_rads_s",
            # "euler_rate_pitch_rads_s",
        }

        # Convert dataclass to dictionary for compact EMA update
        control_input_dict = asdict(control_input)
        # Apply Exponential Moving Average (EMA)
        if self.ema_control_input is None:
            self.ema_control_input = control_input  # Initialize EMA with first input
        else:
            ema_dict = asdict(self.ema_control_input)

            # Apply EMA only to selected fields, copy the rest
            for key in control_input_dict:
                if key in ema_fields:
                    ema_dict[key] = (
                        self.ema_alpha * ema_dict[key]
                        + (1 - self.ema_alpha) * control_input_dict[key]
                    )
                else:
                    ema_dict[key] = control_input_dict[
                        key
                    ]  # Direct copy for non-smoothed fields

            # Convert updated dictionary back to ControlInput dataclass
            self.ema_control_input = ControlInput(**ema_dict)

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

    # TODO: Remove this use for now
    async def _handle_power_command(self, command: str, value: bool):
        """
        Handles power on/off commands for the robot.

        Args:
            value (bool): True to power on the robot, False to power off.
        """
        # Ensure the value is a boolean
        if isinstance(value, bool):
            if value:
                if self.motor_config is not EnabledMotors.NONE:
                    # logger.info(f"Motors power enabled for {str(self.xmotor)}.")
                    logger.info("Motors power enabled for all motors.")
                else:
                    logger.info(f"No motor to turn on.")
            else:
                # Code to execute if the power command is False (e.g., turn off the robot)
                if self.motor_config is not EnabledMotors.NONE:
                    for motor in self.motors:
                        if motor is not None:
                            await motor.stop()
                    # logger.info(f"Motor power disabled to {str(self.xmotor)}.")
                    logger.info("Motor power disabled to all motors.")
                else:
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
