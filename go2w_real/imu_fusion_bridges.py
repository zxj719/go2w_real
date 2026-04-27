"""Helpers for IMU messages used by the RF2O/EKF fusion pipeline."""


def _as_float_list(values, expected_len, name):
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    return result


def covariance_diagonal(x_value, y_value, z_value):
    return [
        float(x_value),
        0.0,
        0.0,
        0.0,
        float(y_value),
        0.0,
        0.0,
        0.0,
        float(z_value),
    ]


def convert_lowstate_imu_sample(
    *,
    quaternion_wxyz,
    gyroscope,
    accelerometer,
    gyro_bias,
    frame_id,
    orientation_covariance_yaw,
    angular_velocity_covariance_z,
    linear_acceleration_covariance,
):
    """Convert Unitree LowState.imu_state fields into a ROS Imu payload.

    Unitree publishes the quaternion as w, x, y, z; ROS Imu uses x, y, z, w.
    """

    quat = _as_float_list(quaternion_wxyz, 4, "quaternion_wxyz")
    gyro = _as_float_list(gyroscope, 3, "gyroscope")
    accel = _as_float_list(accelerometer, 3, "accelerometer")
    bias = _as_float_list(gyro_bias, 3, "gyro_bias")

    return {
        "frame_id": str(frame_id),
        "orientation_xyzw": [quat[1], quat[2], quat[3], quat[0]],
        "angular_velocity": [
            gyro[0] - bias[0],
            gyro[1] - bias[1],
            gyro[2] - bias[2],
        ],
        "linear_acceleration": accel,
        "orientation_covariance": covariance_diagonal(
            999.0, 999.0, orientation_covariance_yaw
        ),
        "angular_velocity_covariance": covariance_diagonal(
            999.0, 999.0, angular_velocity_covariance_z
        ),
        "linear_acceleration_covariance": covariance_diagonal(
            linear_acceleration_covariance,
            linear_acceleration_covariance,
            linear_acceleration_covariance,
        ),
    }


def adapt_utlidar_imu_sample(
    *,
    orientation_xyzw,
    angular_velocity,
    linear_acceleration,
    axis_signs,
    frame_id,
    orientation_covariance_yaw,
    angular_velocity_covariance_z,
    linear_acceleration_covariance,
):
    """Apply the estimated utlidar_imu -> base axis signs to an IMU sample."""

    quat = _as_float_list(orientation_xyzw, 4, "orientation_xyzw")
    gyro = _as_float_list(angular_velocity, 3, "angular_velocity")
    accel = _as_float_list(linear_acceleration, 3, "linear_acceleration")
    signs = _as_float_list(axis_signs, 3, "axis_signs")

    return {
        "frame_id": str(frame_id),
        "orientation_xyzw": quat,
        "angular_velocity": [
            gyro[0] * signs[0],
            gyro[1] * signs[1],
            gyro[2] * signs[2],
        ],
        "linear_acceleration": [
            accel[0] * signs[0],
            accel[1] * signs[1],
            accel[2] * signs[2],
        ],
        "orientation_covariance": covariance_diagonal(
            999.0, 999.0, orientation_covariance_yaw
        ),
        "angular_velocity_covariance": covariance_diagonal(
            999.0, 999.0, angular_velocity_covariance_z
        ),
        "linear_acceleration_covariance": covariance_diagonal(
            linear_acceleration_covariance,
            linear_acceleration_covariance,
            linear_acceleration_covariance,
        ),
    }
