from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional


class TelemetryData(ABC):
    """
    Abstract base class for telemetry data. All telemetry sent identifying a topic,
    and with the data fields as the labels for the data.
    """

    @property
    @abstractmethod
    def topic(self) -> str:
        pass

    def get_data(self):
        """
        Returns a dictionary of the telemetry data excluding the topic.
        Uses asdict() if the subclass is a dataclass.
        """
        return asdict(self)  # Convert to dictionary without the topic

    def to_mqtt(self):
        """
        Returns the topic and the data as a tuple (topic, data).
        The data does not include the topic field.
        """
        return (self.topic, self.get_data())  # Return tuple for sending to MQTT


@dataclass(frozen=True)
class IMUData(TelemetryData):
    """
    Data returned from the IMU sensor

    Units
    -----
    accel_x, accel_y, accel_z: G's
    gyro_x, gyro_y, gyro_z: degrees per second
    temp: Celsius
    mag_x, mag_y, mag_z: uT
    """

    id: int
    accel_x: float  # in G's
    accel_y: float
    accel_z: float
    gyro_x: float  # in degrees per second
    gyro_y: float
    gyro_z: float
    temp: Optional[float] = None  # in degrees Celsius
    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None
    mag_flag: Optional[int] = None  # 0 if data is valid

    @property
    def topic(self) -> str:
        return f"robot/sensors/imu{self.id}"  # Class-level topic

    def get_accel(self) -> tuple:
        return (self.accel_x, self.accel_y, self.accel_z)

    def get_gyro(self) -> tuple:
        return (self.gyro_x, self.gyro_y, self.gyro_z)

    def get_mag(self) -> tuple:
        return (self.mag_x, self.mag_y, self.mag_z)


@dataclass(frozen=True)
class EulerAngles(TelemetryData):
    """
    Data related to AHRS system Euler angles. Order listed is reversed order of rotation

    Units
    -----
    y: degrees
    x: degrees
    z: degrees

    y_dot: radians
    x_dot: radians
    z_dot: radians
    """

    z: float
    x: float
    y: float  # Degrees

    x_dot: float
    y_dot: float
    z_dot: float

    def __post_init__(self):
        # Force conversion to Python's native float
        object.__setattr__(self, "x", float(self.x))
        object.__setattr__(self, "y", float(self.y))
        object.__setattr__(self, "z", float(self.z))

    @property
    def topic(self) -> str:
        return "robot/state/angles"


@dataclass(frozen=True)
class MotorState(TelemetryData):
    """Data synchronized with the motor class.

    Units
    -----
    position: revs
    velocity: revs/s
    torque: Nm
    q_current: A
    d_current: A
    voltage: V
    temperature: Celsius
    """

    name: str  # Motor name for topic differentiation (e.g., 'roll', 'pitch', 'yaw')
    mode: int
    position: float
    velocity: float
    torque: float
    q_current: float
    d_current: float
    voltage: float
    temperature: float
    fault: int

    @property
    def topic(self) -> str:
        """Dynamically generates the topic based on motor name."""
        return f"robot/motor/{self.name}"

    @staticmethod
    def from_dict(data: dict, name: str):
        """Creates a MotorState instance from a dictionary, ensuring key consistency.

        Args:
            data (dict): Dictionary containing motor telemetry data.
            name (str): Name of the motor (e.g., 'roll', 'pitch', 'yaw').

        Returns:
            MotorState: An instance of the MotorState class.
        """
        # Convert dictionary keys to lowercase to standardize input format
        data = {k.lower(): v for k, v in data.items()}
        return MotorState(name=name, **data)


class RollMotorState(MotorState):
    @property
    def topic(self) -> str:
        return "robot/motor_roll"


class PitchMotorState(MotorState):
    @property
    def topic(self) -> str:
        return "robot/motor_pitch"


class YawMotorState(MotorState):
    @property
    def topic(self) -> str:
        return "robot/motor_yaw"


@dataclass(frozen=True)
class ControlData(TelemetryData):
    """Data related to control actions and performance.

    Units
    -----
    loop_time: seconds
    torque_request: Nm
    """

    loop_time: float
    torque_roll: float
    torque_pitch: float
    torque_yaw: float

    @property
    def topic(self) -> str:
        return "robot/control/output"


@dataclass()
class LoopTimer(TelemetryData):
    """
    Data related to control actions and performance.

    Units
    -----
    All fields are measured in seconds.
    """

    imu_read: Optional[float] = None
    sensor_fusion: Optional[float] = None
    motor_states: Optional[float] = None
    control_decision: Optional[float] = None
    duty_cycle_delay_time: Optional[float] = None
    torque_application: Optional[float] = None
    end_loop_buffer: Optional[float] = None

    @property
    def topic(self) -> str:
        return "robot/control/loop_time"

    def convert_to_periods(self):
        """
        Convert the loop times to time intervals
        """
        self.torque_application = self.torque_application - self.duty_cycle_delay_time
        self.duty_cycle_delay_time = self.duty_cycle_delay_time - self.control_decision
        self.control_decision = self.control_decision - self.motor_states
        self.motor_states = self.motor_states - self.sensor_fusion
        self.sensor_fusion = self.sensor_fusion - self.imu_read
        self.imu_read = self.imu_read


@dataclass
class DebugData(TelemetryData):
    """
    A special telemetry data class for debugging purposes.

    Initializes with an empty dictionary for data. To add data, use the add_data method.

    e.g. DebugData().add_data(key1=value1, key2=value2)
    """

    data: Dict[str, Any] = field(default_factory=dict)

    @property
    def topic(self) -> str:
        return "robot/debug"

    @staticmethod
    def from_kwargs(**kwargs):
        return DebugData(data=kwargs)

    def add_data(self, **kwargs):
        """Update the data dictionary with new key-value pairs."""
        self.data.update(kwargs)

    def get_data(self):
        """
        Override to return the `data` dictionary directly,
        instead of wrapping it in another dictionary.
        """
        return self.data
