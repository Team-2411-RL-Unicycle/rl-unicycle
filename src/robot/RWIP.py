import time
from icm20948.lib.imu_lib import ICM20948
from fusion.AHRSfusion import AHRSfusion
import robot.LoopTimer as lt
from motors.MN6007 import MN6007
from controller import (
    ControlInput,
    Controller,
    PIDController,
    RLController,
    TestController,
)
import logging

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
        robot_io (RobotIO): The RobotIO instance for handling input/output operations.
        imu (ICM20948): The IMU sensor instance.
        sensor_fusion (AHRSfusion): The sensor fusion algorithm instance for processing IMU data.
        xmotor (MN6007 or None): The motor controller instance, if motors are started.
        itr (int): An iteration counter for the control loop.

    Methods:
        start(): Initializes actuators and starts the control loop.
        control_loop(): The main control loop for the robot, executed at a fixed rate.
        shutdown(): Safely shuts down the robot system, including sensors and actuators.
        handle_power_command(value: bool): Handles power on/off commands.
        handle_pid_command(value): Handles PID control commands.
        precise_delay_until(end_time): Delays execution until a specified end time to maintain loop timing.
    """

    LOOP_TIME = 1 / 100  # 100 Hz control loop period
    WRITE_DUTY = 0.6  # Percent of loop time passed before write to actuators
    MAX_TORQUE = 0.6  # Maximum torque for motor torque (testing purposes)

    def __init__(
        self, send_queue, receive_queue, start_motors=True, controller_type="test"
    ):
        # Setup the communication queues and the input output over internet system
        self.robot_io = RobotIO(
            send_queue,
            receive_queue,
            {
                "power": self.handle_power_command,
                "P": self.handle_pid_command,
                "I": self.handle_pid_command,
                "D": self.handle_pid_command,
            },
        )

        # Init the IMU from library Config files are found in icm20948/configs dir and keep the imu settings
        imu1 = "imu1.ini"
        self.imu = ICM20948(config_file=imu1)
        # Init sensor fusion
        self.sensor_fusion = AHRSfusion(self.imu._gyro_range, int(1 / self.LOOP_TIME))
        self.sensor_calibration_delay = 5  # seconds

        self.motors_enabled = start_motors
        # Initialize all actuators
        if start_motors:
            self.xmotor = MN6007()
        else:
            self.xmotor = None

        # Initialize controller type and instantiate the appropriate controller based on the controller_type argument
        self.controller_type = controller_type
        if controller_type == "pid":
            # TODO test PID
            self.controller = PIDController()
        elif controller_type == "rl":
            self.controller = RLController(model_pth="controller/rwip_model_m22.onnx")
        elif controller_type == "test":
            self.controller = TestController()
        else:
            raise ValueError(f"Unsupported controller type: {controller_type}")

        self.itr = int(0)  # Cycle counter

    async def start(self):
        """
        Initializes actuators and starts the control loop.
        """
        # Init actuators
        if self.xmotor is not None:
            await self.xmotor.start()
        await self.control_loop()

    async def control_loop(self):
        """
        The main control loop for the robot. Executes sensor reading, state estimation, control decision making,
        and actuator commands at a fixed rate defined by LOOP_TIME.
        """
        loop_period = 0.01
        while True:
            # Start Loop Timer and increment loop iteration
            loop_start_time = time.time()
            self.itr += 1

            # Poll the imu for positional data
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro(convert=True)

            # Fuse sensor data
            euler_angles, internal_states, flags = self.sensor_fusion.update(
                (gx, gy, gz), (ax, ay, az), delta_time=loop_period
            )

            # Update robot state and parameters
            if self.xmotor is not None:
                await self.xmotor.update_state()

            control_input = ControlInput(
                pendulum_angle=euler_angles[1],  # euler y (robot frame)
                pendulum_vel=gz,  # gyro z (imu frame angular speed, gyro y in robot frame)
                wheel_vel=0 if self.xmotor is None else self.xmotor.state["VELOCITY"],
            )

            # Change to negative convention due to motor
            torque_request = -self.controller.get_torque(
                control_input, self.MAX_TORQUE - 0.001
            )  # Floating point buffer
            self.robot_io.send_debug_data(torque_request=float(torque_request))

            ## DELAY UNTIL FIXED POINT ##
            self.precise_delay_until(loop_start_time + loop_period * self.WRITE_DUTY)

            # Apply control decision to robot actuators
            # SET TORQUE
            isCalibrating = self.itr < self.sensor_calibration_delay / self.LOOP_TIME
            if self.motors_enabled and not isCalibrating:
                # Only set the torque if not in calibration mode
                await self.xmotor.set_torque(
                    torque=torque_request, max_torque=self.MAX_TORQUE
                )

            ### SEND COMMS ###
            # Send out all data downsampled to lower rate
            if (self.itr % 20) == 0:
                self.robot_io.send_imu_data(ax, ay, az, gx, gy, gz)
                self.robot_io.send_euler_angles(euler_angles)
                if self.xmotor is not None:
                    self.robot_io.send_motor_state(self.xmotor.state)

            # High Frequency data
            self.robot_io.send_loop_time(loop_period * 1e6)
            if self.xmotor is not None:
                self.robot_io.send_motor_electrical(
                    q_current=self.xmotor.state["Q_CURRENT"],
                    d_current=self.xmotor.state["D_CURRENT"],
                    torque=self.xmotor.state["TORQUE"],
                    voltage=self.xmotor.state["VOLTAGE"],
                    velocity=self.xmotor.state["VELOCITY"],
                    motor_fault=self.xmotor.state["FAULT"],
                )

            ### RECEIVE COMMS ###
            # Check for and handle new commands sent in via MQTT
            await self.robot_io.receive_commands()

            ### CLOSE LOOP DELAY ###
            end_time = loop_start_time + RobotSystem.LOOP_TIME
            loop_delay = self.precise_delay_until(end_time)
            loop_period = time.time() - loop_start_time

    async def shutdown(self):
        """
        Safely shuts down the robot system, including closing sensor interfaces and stopping actuators.
        """
        self.imu.close()
        if self.xmotor is not None:
            await self.xmotor.stop()

    async def handle_power_command(self, command: str, value: bool):
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

    async def handle_pid_command(self, command: str, value: float):
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
        # Code to execute if the PID value is a float (e.g., set value)
        # Command comes in as P, I, or D
        # This likely will be offloaded to the PID controller if one is enabled
        self.controller.update_parameter(command, value)
        # Log info and print to console
        msg = f"Setting PID parameter {command} to value {value}."
        logger.info(msg)
        print(msg)
        return

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


class RobotIO:  #
    """
    The RobotIO class handles all input/output operations for the robot, including sending sensor data,
    state information, and receiving and processing commands.

    Attributes:
        send_queue: The queue used for sending data and messages to other system components.
        receive_queue: The queue used for receiving commands and messages from other system components.
        command_callbacks (dict): A dictionary mapping command strings to their handling methods.
    """

    def __init__(self, send_queue, receive_queue, callback: dict) -> None:
        self.send_queue = send_queue
        self.receive_queue = receive_queue
        self.command_callbacks = callback

    def send_debug_data(self, **kwargs):
        """
        Sends debug data with flexible key-value pairs.

        Each key-value pair in kwargs represents a label and its corresponding value.
        The function adds a timestamp to the data before sending it off in the queue.

        Example usage:
            robot_io.send_debug_data(motor_temp=85, battery_voltage=12.6)

        Args:
            **kwargs: Arbitrary number of named arguments representing debug data labels and values.
        """
        debug_data = {
            **kwargs  # Merge the arbitrary key-value pairs into the debug data
        }
        # Define the topic for debug data
        topic = "robot/debug"

        # Put the debug data into the send queue
        self.send_queue.put((topic, debug_data))

    def send_motor_electrical(self, **kwargs):
        debug_data = {
            **kwargs  # Merge the arbitrary key-value pairs into the debug data
        }
        # Define the topic for debug data
        topic = "robot/sensors/motor_electrical"
        # Put the debug data into the send queue
        self.send_queue.put((topic, debug_data))

    def send_imu_data(self, ax, ay, az, gx, gy, gz):
        topic = "robot/sensors/imu"
        data = {
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
        }
        self.send_queue.put((topic, data))

    def send_euler_angles(self, euler_angles):
        topic = "robot/state/angles"
        data = {
            "euler_x": float(euler_angles[0]),  # Convert to Python float
            "euler_y": float(euler_angles[1]),
            "euler_z": float(euler_angles[2]),
        }
        self.send_queue.put((topic, data))

    def send_motor_state(self, motor_state):
        topic = "robot/sensors/motor"
        data = motor_state
        self.send_queue.put((topic, data))

    def send_loop_time(self, loop_time):
        topic = "robot/control/loop_time"
        data = {"loop_time": loop_time}
        self.send_queue.put((topic, data))

    async def receive_commands(self):
        """
        Processes commands received in the receive queue from MQTT. Each message contains
        a single command-value pair. The corresponding handler function in the
        `command_callbacks` dictionary is called with the command and its value.

        If the command does not have a registered handler in `command_callbacks`, an error is logged indicating
        that the command is unrecognized.
        """
        while not self.receive_queue.empty():
            message = self.receive_queue.get()

            # Directly extract command and value from the message assuming it's a single-entry dict
            if isinstance(message, dict) and len(message) == 1:
                command, value = next(iter(message.items()))

                if command in self.command_callbacks:
                    handler = self.command_callbacks[command]
                    await handler(
                        command, value
                    )  # Invoke the handler with the command value
                else:
                    logger.error(f"Unrecognized command: {command}")
            else:
                logger.error(
                    f"Expected a single-entry dictionary for the message, got: {message}"
                )

            self.receive_queue.task_done()
