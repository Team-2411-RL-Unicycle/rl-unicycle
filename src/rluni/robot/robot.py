import asyncio
import logging
import math
import time
from collections import namedtuple
from dataclasses import asdict
from datetime import datetime
from enum import Enum
from importlib.resources import files
from multiprocessing import Queue
from typing import Callable, List, Tuple, Union

import moteus
import numpy as np

from rluni.controller.fullrobot import (ControlInput, Controller,
                                        LQRController, HighLevelXboxController,
                                        MPCController, RLController,
                                        TestController)
from rluni.fusion.AHRSfusion import AHRSfusion
from rluni.icm20948.imu_lib import ICM20948
from rluni.motors.motors import MN2806, MN6007, Motor
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

from rluni.utils.csv_logger import CSVLogger
from . import safety_buffer as sb
from . import teledata as td

logger = logging.getLogger(__name__)

DEG_TO_RAD = math.pi / 180
REV_TO_RAD = 2 * math.pi


class EnabledMotors(Enum):
    NONE = 0
    ROLL = 1
    YAW = 2
    PITCH = 3
    ROLL_PITCH = 4
    ALL = 5
    NUM_CONFIGS = 6


MotorsTuple = namedtuple("motors", ["roll", "pitch", "yaw"])


class RobotSystem:
    """
    The RobotSystem class manages core robot functions including:
      - Sensors (IMUs)
      - Sensor fusion
      - Control loop
      - Motors and torque commands
      - Command handling via queues
      - Telemetry publishing

    Attributes:
        LOOP_TIME (float): The fixed period for the control loop in seconds.
        WRITE_DUTY (float): The fraction of the loop period before actuators are updated.
        MAX_TORQUE_ROLL_PITCH (float): Max torque used for roll/pitch motors.
        MAX_TORQUE_YAW (float): Max torque used for yaw motor.
        itr (int): Loop counter.
    """

    def __init__(
        self,
        send_queue: Queue,
        receive_queue: Queue,
        run_motors: bool = True,
        controller_type: str = "test",
        config_file=None,
        motor_config=None,
    ):
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        # Load and parse the configuration file
        self._load_config(config_file)
        self.safety_buffer = sb.SafetyBuffer()

        # Initialize IMU and sensor fusion
        self.imu1 = ICM20948(config_file=self.imu_config1)
        self.imu2 = ICM20948(config_file=self.imu_config2)
        self.sensor_fusion1 = AHRSfusion(
            sample_rate=int(1 / self.LOOP_TIME), config_file=self.imu_config1
        )
        self.sensor_fusion2 = AHRSfusion(
            sample_rate=int(1 / self.LOOP_TIME), config_file=self.imu_config2
        )

        self.motors: Tuple[Motor, ...] = None
        self.motor_config = EnabledMotors.NONE
        self._initialize_motors(motor_config)
        self.run_motors = run_motors

        self.controller_type = controller_type
        self.controller = self._get_controller(controller_type)

        # For exponential smoothing of certain fields in control input
        self.ema_control_input = None        

        self.itr = int(0)  # Cycle counter

        if self.csv_logging_enabled:
            self.csv_logger = CSVLogger()
            self.csv_logger.open()

    def _load_config(self, config_file):
        """Load the robot configuration file and set relevant parameters."""
        # Load the robot configuration file (use the default if none provided)
        if config_file is None:
            config_file = "unicycle.yaml"
            logger.warning(
                f"No configuration file provided. Using default: {config_file}"
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

        self.ema_alpha = gvcv(config, "RobotSystem.ema_alpha", float, required=False)
        self.ema_alpha = 0.60 if self.ema_alpha is None else self.ema_alpha

        self.imu1_weight = gvcv(config, "RobotSystem.imu1_weight", float, required=False)
        self.imu1_weight = 0.5 if self.imu1_weight is None else self.imu1_weight

        self.rlmodel_path = gvcv(
            config, "RobotSystem.rlmodel_path", str, required=False
        )
        if self.rlmodel_path is not None:
            self.rlmodel_path = str(files("rluni").joinpath(self.rlmodel_path))
            
        self.csv_logging_enabled = gvcv(config, "RobotSystem.csv_logging", bool, required=True)

    def _initialize_motors(self, motor_config):
        """Initialize motors based on string 'motor_config'."""
        self.transport = moteus.Fdcanusb(
            "/dev/serial/by-id/usb-mjbots_fdcanusb_5A70499D-if00"
        )

        # Decide which motors to enable
        if motor_config == "none" or motor_config is None:
            self.motor_config = EnabledMotors.NONE
            self.motors = MotorsTuple(None, None, None)

        elif motor_config == "roll":
            self.motor_config = EnabledMotors.ROLL
            self.motors = MotorsTuple(
                MN6007(4, "roll", self.transport),
                None,
                None,
            )

        elif motor_config == "yaw":
            self.motor_config = EnabledMotors.YAW
            self.motors = MotorsTuple(
                None,
                None,
                MN2806(7, "yaw", self.transport),
            )

        elif motor_config == "pitch":
            self.motor_config = EnabledMotors.PITCH
            self.motors = MotorsTuple(
                None,
                MN6007(6, "pitch", self.transport),
                None,
            )

        elif motor_config in ("roll_pitch", "pitch_roll"):
            self.motor_config = EnabledMotors.ROLL_PITCH
            self.motors = MotorsTuple(
                MN6007(4, "roll", self.transport),
                MN6007(6, "pitch", self.transport),
                None,
            )

        elif motor_config == "all":
            self.motor_config = EnabledMotors.ALL
            self.motors = MotorsTuple(
                MN6007(4, "roll", self.transport),
                MN6007(6, "pitch", self.transport),
                MN2806(7, "yaw", self.transport),
            )

        else:
            logger.warning("No valid motor configuration provided. Disabling motors.")
            self.motors = MotorsTuple(None, None, None)
            self.motor_config = EnabledMotors.NONE

    def _get_controller(self, controller_type: str) -> Controller:
        """Initialize the controller based on the type ('rl', 'lqr', 'test', 'xbox')."""
        if controller_type == "rl":
            return RLController(model_pth=self.rlmodel_path)
        elif controller_type == "lqr":
            return LQRController()
        elif controller_type == "mpc":
            return MPCController(dt=self.LOOP_TIME)
        elif controller_type == "test":
            return TestController()
        elif controller_type == "xbox":
            return HighLevelXboxController()
        else:
            raise ValueError(f"Unsupported controller type: {controller_type}")

    """ ##################################################################
    Public Lifecycle Methods
    ################################################################## """

    async def start(self, shutdown_event: asyncio.Event):
        """
        Start the main control loop, continue until `shutdown_event` is set.
        """
        try:
            await self.control_loop(shutdown_event)
        except asyncio.CancelledError:
            logger.info("Robot Loop received shutdown signal.")
            shutdown_event.set()
            raise

    async def shutdown(self):
        """
        Shutdown the robot system and its components.
        """
        try:
            for motor in self.motors:
                if motor is not None:
                    await motor.shutdown()
            logger.info("Motors shut down successfully.")

            if self.csv_logging_enabled and hasattr(self, "csv_logger"):
                self.csv_logger.close()

        except Exception as e:
            logger.exception(f"Error during RobotSystem shutdown: {e}")

    """ ##################################################################
    Main Control Loop
    ################################################################## """

    async def control_loop(self, shutdown_event):
        """
        Core robot control loop. Executes at a fixed rate defined by LOOP_TIME:
          1) Read sensors
          2) Update fusion
          3) Read motor states
          4) Calculate control input
          5) Apply torque if safe and not calibrating
          6) Send telemetry
          7) Handle commands
          8) Sleep until next iteration
        """
        loop_period = self.LOOP_TIME

        while not shutdown_event.is_set():
            # Start Loop Timer and increment loop iteration
            loop_start_time = time.time()
            self.itr += 1

            # Create a timing/telemetry structure to measure performance
            timer_tele = td.LoopTimer()

            # Debugging Data Example (add more data as needed)
            tele_debug_data = td.DebugData()
            tele_debug_data.add_data(example_debug_data=99)

            # ---- 1) Read raw sensor data ----
            imudata1, imudata2 = self._read_sensors()
            timer_tele.imu_read = time.time() - loop_start_time

            # ---- 2) Update sensor fusion to get orientation ----
            quaternions, eulers_deg_list, euler_rates_rads_list = (
                self._update_sensor_fusion(imudata1, imudata2, loop_period)
            )
            # Combine estimates from both sensors (list of quaternions, eulers, rates)
            w1 = self.imu1_weight
            w2 = 1.0-w1
            eulers_deg = w1 * eulers_deg_list[0] + w2 * eulers_deg_list[1]
            euler_rates_rads = w1 * euler_rates_rads_list[0] + w2 * euler_rates_rads_list[1]    
            
            timer_tele.sensor_fusion = time.time() - loop_start_time

            # ---- 3) Update motor states ----
            # Pass iter to do async update of single motor
            await self._update_motors(iter = self.itr)
            timer_tele.motor_states = time.time() - loop_start_time

            # ---- 4) Build control input (and optionally smooth it) ----
            control_input, rigid_body_state = self._calculate_control_input(
                eulers_deg, euler_rates_rads
            )

            # Evaluate system safety
            safe_state, safe_msg = self.safety_buffer.evaluate_state(control_input)
            if not safe_state:
                logger.warning(f"Robot is not in a safe state: {safe_msg}")
                shutdown_event.set()

            self._update_ema_control_input(control_input)

            # ---- 5) Obtain torques from the controller ----
            # TODO: This needs updating to specify multiple torque limits
            torques = self.controller.get_torques(
                self.ema_control_input, self.MAX_TORQUE_ROLL_PITCH - 0.001
            )
            timer_tele.control_decision = time.time() - loop_start_time

            # ---- Wait until Write Duty point to apply torque ----
            self._precise_delay_until(loop_start_time + loop_period * self.WRITE_DUTY)
            timer_tele.duty_cycle_delay_time = time.time() - loop_start_time

            # ---- Apply torques if not calibrating ----
            is_calibrating = self.itr < self.sensor_calibration_delay / self.LOOP_TIME
            if (
                not is_calibrating
                and (self.motor_config is not EnabledMotors.NONE)
                and self.run_motors
            ):
                await self._apply_control(torques)

            timer_tele.torque_application = time.time() - loop_start_time

            # ---- 6) Send telemetry (downsample if desired) ----
            if self.csv_logging_enabled:
                self._log_to_csv(
                    control_input,
                    torques,
                    is_calibrating,
                    timestamp=datetime.now().isoformat(),
                )

            # Convert loop timer intervals to periods, track leftover time
            timer_tele.convert_to_periods()
            timer_tele.end_loop_buffer = self.LOOP_TIME - (
                time.time() - loop_start_time
            )

            if (self.itr % 1) == 0:
                await self._send_telemetry(
                    imudata1,
                    imudata2,
                    rigid_body_state,
                    torques,
                    timer_tele,
                )

            ### RECEIVE COMMS ###
            # Check for and handle new commands sent in via MQTT
            await self._handle_commands()

            # ---- 8) Sleep to maintain loop rate ----
            end_time = loop_start_time + self.LOOP_TIME
            self._precise_delay_until(end_time)
            loop_period = time.time() - loop_start_time

    """ ##################################################################
    Helper Methods (Reading Sensors, Updating Motors, Commands, etc.)
    ################################################################## """

    def _read_sensors(self) -> Tuple[td.IMUData, td.IMUData]:
        """
        Read raw data from IMUs and return them as TelemetryData objects.
        """
        data1 = self.imu1.read_sensor_data(convert=True)
        data2 = self.imu2.read_sensor_data(convert=True)
        imudata1 = td.IMUData(1, *data1)
        imudata2 = td.IMUData(2, *data2)
        return imudata1, imudata2

    def _update_sensor_fusion(
        self, imudata1: td.IMUData, imudata2: td.IMUData, delta_time: float
    ) -> Tuple[List[float], List[float], List[float]]:
        """
        Update sensor fusion using the first IMU’s data (or fuse with second if desired).
        Returns:
            (quaternion, euler_angles, euler_rates)
        """
        imus = [imudata1, imudata2]
        fusions = [self.sensor_fusion1, self.sensor_fusion2]

        quaternions = []
        eulers_all = []
        euler_rates_all = []

        for imu, fusion in zip(imus, fusions):

            gyro = imu.get_gyro()
            accel = imu.get_accel()
            mag = imu.get_mag()

            quaternion, internal_states, flags = fusion.update(
                gyro_data=gyro,
                accel_data=accel,
                mag_data=mag,
                delta_time=delta_time,
            )

            # Euler angles and rates are stored in self.sensor_fusion
            eulers = fusion.euler_angles  # DEGREES
            euler_rates = fusion.euler_rates  # RADS/S

            quaternions.append(quaternion)
            eulers_all.append(eulers)
            euler_rates_all.append(euler_rates)

        return quaternions, eulers_all, euler_rates_all
    
    async def _update_motors(self, iter = None):        
        """Asynchronously query each motor to update its internal state."""
        n_motors = len(self.motors)

        # Update a single motor
        if iter is not None and (iter > 5):
            motor_idx = iter % n_motors
            motor = self.motors[motor_idx]
            if motor is not None:
                await motor.update_state()
        
        # Update all motors        
        else:        
            for motor in self.motors:
                if motor is not None:
                    await motor.update_state()

    def _calculate_control_input(
        self,
        eulers_deg: Tuple[float, float, float],
        euler_rates_rads: Tuple[float, float, float],
    ) -> Tuple[ControlInput, td.EulerAngles]:
        """
        Convert sensor-fusion euler angles/rates into the `ControlInput` structure
        used by the controllers, and also build a Telemetry structure (EulerAngles).
        """
        roll_deg, pitch_deg, yaw_deg = eulers_deg
        roll_rate, pitch_rate, yaw_rate = euler_rates_rads

        # Build EulerAngles for telemetry
        rigid_body_state = td.EulerAngles(
            roll_deg,
            pitch_deg,
            yaw_deg,
            roll_rate,
            pitch_rate,
            yaw_rate,
        )

        # Build ControlInput
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
            motor_position_pitch_rads=(
                0.0
                if self.motors.pitch is None
                else self.motors.pitch.state["POSITION"] * REV_TO_RAD
            ),
        )

        return control_input, rigid_body_state

    def _update_ema_control_input(self, control_input: ControlInput):
        """
        Exponential Moving Average on selected fields for smoother control.
        """
        fields_to_smooth = {"euler_rate_roll_rads_s"}

        # Convert dataclass to dictionary
        current_dict = asdict(control_input)

        if self.ema_control_input is None:
            self.ema_control_input = control_input
        else:
            ema_dict = asdict(self.ema_control_input)
            for key, val in current_dict.items():
                if key in fields_to_smooth:
                    ema_dict[key] = (
                        self.ema_alpha * ema_dict[key] + (1 - self.ema_alpha) * val
                    )
                else:
                    ema_dict[key] = val

            # Convert back to a dataclass
            self.ema_control_input = ControlInput(**ema_dict)

    def _log_to_csv(self, control_input, torques, is_calibrating: bool, timestamp: str):
        """
        Log the current state and control actions to the CSV file.

        Args:
            control_input: The current control input state.
            torques: The torques applied to the motors.
            is_calibrating: Whether the robot is calibrating.
            timestamp: The current timestamp.
        """
        if not hasattr(self, "csv_logger"):
            logger.warning("CSV logger not initialized")
            return

        # Create a dictionary with all the data we want to log
        data = {
            "timestamp": timestamp,
            "iteration": self.itr,
            "roll_rads": control_input.euler_angle_roll_rads,
            "pitch_rads": control_input.euler_angle_pitch_rads,
            "yaw_rads": control_input.euler_angle_yaw_rads,
            "roll_rate_rads": control_input.euler_rate_roll_rads_s,
            "pitch_rate_rads": control_input.euler_rate_pitch_rads_s,
            "yaw_rate_rads": control_input.euler_rate_yaw_rads_s,
            "motor_speed_roll_rads": control_input.motor_speeds_roll_rads_s,
            "motor_speed_pitch_rads": control_input.motor_speeds_pitch_rads_s,
            "motor_speed_yaw_rads": control_input.motor_speeds_yaw_rads_s,
            "motor_position_pitch_rads": control_input.motor_position_pitch_rads,
            "torque_roll": float(torques.roll) if hasattr(torques, "roll") else 0.0,
            "torque_pitch": float(torques.pitch) if hasattr(torques, "pitch") else 0.0,
            "torque_yaw": float(torques.yaw) if hasattr(torques, "yaw") else 0.0,
            "is_calibrating": int(is_calibrating),
        }

        # Log the data
        self.csv_logger.log(data)

    async def _apply_control(self, torques):
        """
        Build and send the torque commands to each motor via the transport.
        """
        commands = []
        if self.motors.roll is not None:
            commands.append(
                self.motors.roll._c.make_position(
                    position=math.nan,
                    kp_scale=0.0,
                    kd_scale=0.0,
                    feedforward_torque=torques.roll,
                    maximum_torque=self.MAX_TORQUE_ROLL_PITCH - 0.001,
                    watchdog_timeout=0.1
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
                    watchdog_timeout=0.1
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
                    watchdog_timeout=0.1
                )
            )

        await self.transport.cycle(commands)

    def _precise_delay_until(self, end_time):
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

    """ ##################################################################
        Telemetry & Command Handling
        ################################################################## """

    async def _send_telemetry(
        self,
        imudata1: td.IMUData,
        imudata2: td.IMUData,
        rigid_body_state: td.EulerAngles,
        torques,
        timer_tele: td.LoopTimer,
    ):
        """
        Compose telemetry data objects and send them via the `send_queue`.
        """
        control_data = td.ControlData(
            loop_time=self.LOOP_TIME,
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
        ]

        # Include motor states
        for motor in self.motors:
            if motor is not None:
                data_list.append(
                    td.MotorState.from_dict(data=motor.state, name=motor.name)
                )

        self.send_queue.put(data_list)

    async def _handle_commands(self):
        """Handle commands from the receive queue."""
        while not self.receive_queue.empty():
            message = self.receive_queue.get()

            if isinstance(message, dict):
                for command, value in message.items():
                    await self._execute_command(command, value)
            else:
                logger.error(f"Invalid command format: {message}")

    async def _execute_command(self, command: str, value):
        """
        Dispatch commands to either internal RobotSystem handlers (like 'power' or 'controller')
        or pass them on to the active controller if it supports command handling.
        """
        handled = False

        if command == "power":
            handled = True
            await self._handle_power_command(command, value)

        elif command == "controller":
            handled = True
            await self._handle_controller_switch(command, value)

        # Check if a control that listens for commands is active
        if not handled and hasattr(self.controller, "handle_command"):
            try:
                self.controller.handle_command(command, value)
                handled = True
            except Exception as ex:
                logger.error(
                    f"Active controller failed to handle command '{command}': {ex}"
                )

        # 3) If still not handled, log an error or do nothing
        if not handled:
            logger.error(f"Unrecognized or unhandled command: {command}")

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
                    self.run_motors = True
                    logger.info("Motors power enabled for all motors.")
                else:
                    logger.info(f"No motor to turn on.")
            else:
                # Code to execute if the power command is False (e.g., turn off the robot)
                if self.motor_config is not EnabledMotors.NONE:
                    for motor in self.motors:
                        if motor is not None:
                            await motor.stop()
                        self.run_motors = False
                    # logger.info(f"Motor power disabled to {str(self.xmotor)}.")
                    logger.info("Motor power disabled to all motors.")
                else:
                    logger.info("No motor to turn off.")
        else:
            # Log an error or handle the case where the value is not a boolean
            logger.error(f"Expected a boolean for the power command, but got: {value}")

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
