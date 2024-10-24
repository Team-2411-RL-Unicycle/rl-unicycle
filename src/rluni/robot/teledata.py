from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from typing import Any, Dict


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
    """Data returned from the IMU sensor"""

    accel_x: float  # 1.0 = 9.8 m/s^2
    accel_y: float
    accel_z: float
    gyro_x: float  # degrees per second
    gyro_y: float
    gyro_z: float

    @property
    def topic(self) -> str:
        return "robot/sensors/imu"  # Class-level topic

    def get_accel(self) -> tuple:
        return (self.accel_x, self.accel_y, self.accel_z)

    def get_gyro(self) -> tuple:
        return (self.gyro_x, self.gyro_y, self.gyro_z)


@dataclass(frozen=True)
class EulerAngles(TelemetryData):
    """Data related to AHRS system"""

    x: float  # Degrees
    y: float
    z: float

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
    """Data synchronized with the motor class"""

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
        return "robot/motor"

    @staticmethod
    def from_dict(data: dict):
        # Convert dictionary keys to lowercase and pass as kwargs
        data = {k.lower(): v for k, v in data.items()}
        return MotorState(**data)


@dataclass(frozen=True)
class ControlData(TelemetryData):
    """Data related to control actions and performance."""

    loop_time: float
    torque_request: float

    @property
    def topic(self) -> str:
        return "robot/control"


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
