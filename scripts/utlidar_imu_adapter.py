#!/usr/bin/env python3
"""Adapt /utlidar/imu into the estimated base IMU frame for EKF yaw-rate use."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from go2w_real.imu_fusion_bridges import adapt_utlidar_imu_sample
from lowstate_imu_bridge import _fill_imu_msg


class UtlidarImuAdapter(Node):
    def __init__(self):
        super().__init__("utlidar_imu_adapter")
        self.declare_parameter("input_topic", "/utlidar/imu")
        self.declare_parameter("output_topic", "/utlidar_imu_base")
        self.declare_parameter("frame_id", "imu")
        self.declare_parameter("axis_signs", [1.0, 1.0, -1.0])
        self.declare_parameter("orientation_covariance_yaw", 999.0)
        self.declare_parameter("angular_velocity_covariance_z", 0.02)
        self.declare_parameter("linear_acceleration_covariance", 100.0)

        self._frame_id = self.get_parameter("frame_id").value
        self._axis_signs = self.get_parameter("axis_signs").value
        self._orientation_covariance_yaw = float(
            self.get_parameter("orientation_covariance_yaw").value
        )
        self._angular_velocity_covariance_z = float(
            self.get_parameter("angular_velocity_covariance_z").value
        )
        self._linear_acceleration_covariance = float(
            self.get_parameter("linear_acceleration_covariance").value
        )

        self._publisher = self.create_publisher(
            Imu, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self._callback, 50
        )

    def _callback(self, msg):
        payload = adapt_utlidar_imu_sample(
            orientation_xyzw=[
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            angular_velocity=[
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            linear_acceleration=[
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            axis_signs=self._axis_signs,
            frame_id=self._frame_id,
            orientation_covariance_yaw=self._orientation_covariance_yaw,
            angular_velocity_covariance_z=self._angular_velocity_covariance_z,
            linear_acceleration_covariance=self._linear_acceleration_covariance,
        )
        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        _fill_imu_msg(imu_msg, payload)
        self._publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UtlidarImuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
