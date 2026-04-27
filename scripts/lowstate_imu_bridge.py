#!/usr/bin/env python3
"""Publish /lowstate.imu_state as a rate-limited sensor_msgs/Imu topic."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState

from go2w_real.imu_fusion_bridges import convert_lowstate_imu_sample


def _set_covariance(target, values):
    for index, value in enumerate(values):
        target[index] = float(value)


def _fill_imu_msg(msg, payload):
    msg.header.frame_id = payload["frame_id"]
    msg.orientation.x = payload["orientation_xyzw"][0]
    msg.orientation.y = payload["orientation_xyzw"][1]
    msg.orientation.z = payload["orientation_xyzw"][2]
    msg.orientation.w = payload["orientation_xyzw"][3]
    msg.angular_velocity.x = payload["angular_velocity"][0]
    msg.angular_velocity.y = payload["angular_velocity"][1]
    msg.angular_velocity.z = payload["angular_velocity"][2]
    msg.linear_acceleration.x = payload["linear_acceleration"][0]
    msg.linear_acceleration.y = payload["linear_acceleration"][1]
    msg.linear_acceleration.z = payload["linear_acceleration"][2]
    _set_covariance(msg.orientation_covariance, payload["orientation_covariance"])
    _set_covariance(
        msg.angular_velocity_covariance, payload["angular_velocity_covariance"]
    )
    _set_covariance(
        msg.linear_acceleration_covariance,
        payload["linear_acceleration_covariance"],
    )


class LowStateImuBridge(Node):
    def __init__(self):
        super().__init__("lowstate_imu_bridge")
        self.declare_parameter("source_topic", "/lowstate")
        self.declare_parameter("imu_topic", "/lowstate_imu")
        self.declare_parameter("frame_id", "imu")
        self.declare_parameter("gyro_bias", [0.0, -0.00207, -0.00902])
        self.declare_parameter("publish_rate_limit_hz", 100.0)
        self.declare_parameter("orientation_covariance_yaw", 0.05)
        self.declare_parameter("angular_velocity_covariance_z", 0.04)
        self.declare_parameter("linear_acceleration_covariance", 100.0)

        self._frame_id = self.get_parameter("frame_id").value
        self._gyro_bias = self.get_parameter("gyro_bias").value
        self._orientation_covariance_yaw = float(
            self.get_parameter("orientation_covariance_yaw").value
        )
        self._angular_velocity_covariance_z = float(
            self.get_parameter("angular_velocity_covariance_z").value
        )
        self._linear_acceleration_covariance = float(
            self.get_parameter("linear_acceleration_covariance").value
        )
        rate_limit = float(self.get_parameter("publish_rate_limit_hz").value)
        self._min_period_sec = 1.0 / rate_limit if rate_limit > 0.0 else 0.0
        self._last_publish_time = None

        self._publisher = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10
        )
        self.create_subscription(
            LowState, self.get_parameter("source_topic").value, self._callback, 50
        )

    def _callback(self, msg):
        now = self.get_clock().now()
        if self._last_publish_time is not None:
            elapsed = (now - self._last_publish_time).nanoseconds * 1e-9
            if elapsed < self._min_period_sec:
                return
        self._last_publish_time = now

        imu = msg.imu_state
        payload = convert_lowstate_imu_sample(
            quaternion_wxyz=imu.quaternion,
            gyroscope=imu.gyroscope,
            accelerometer=imu.accelerometer,
            gyro_bias=self._gyro_bias,
            frame_id=self._frame_id,
            orientation_covariance_yaw=self._orientation_covariance_yaw,
            angular_velocity_covariance_z=self._angular_velocity_covariance_z,
            linear_acceleration_covariance=self._linear_acceleration_covariance,
        )
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        _fill_imu_msg(imu_msg, payload)
        self._publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowStateImuBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
