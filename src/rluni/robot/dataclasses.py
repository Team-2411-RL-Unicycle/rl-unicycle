from typing import TypedDict


class IMUData(TypedDict):
    """
    IMU data dictionary

    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    """

    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


class EulerAngles(TypedDict):
    euler_x: float
    euler_y: float
    euler_z: float


class MotorState(TypedDict):
    position: float
    velocity: float
    torque: float
    # Add other motor state fields as needed


class MotorElectrical(TypedDict):
    q_current: float
    d_current: float
    torque: float
    voltage: float
    velocity: float
    motor_fault: int


class ControlData(TypedDict):
    loop_time: float


class DebugData(TypedDict):
    torque_request: float


class RobotState(TypedDict):
    imu_data: IMUData
    euler_angles: EulerAngles
    motor_state: MotorState
    motor_electrical: MotorElectrical
    control_data: ControlData
    debug_data: DebugData


def get_accel_tuple(imu_data: IMUData):
    return (imu_data["accel_x"], imu_data["accel_y"], imu_data["accel_z"])


def get_gyro_tuple(imu_data: IMUData):
    return (imu_data["gyro_x"], imu_data["gyro_y"], imu_data["gyro_z"])
