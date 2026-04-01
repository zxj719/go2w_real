#!/usr/bin/env python3
"""
Wait for a TF transform to become available, then exit.

Used by launch files to gate SLAM/Nav2 startup on a valid TF chain.
"""

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class WaitForTransform(Node):
    def __init__(self):
        super().__init__("wait_for_transform")

        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "base")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("poll_period", 0.2)
        self.declare_parameter("log_period", 2.0)

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.timeout_sec = (
            self.get_parameter("timeout_sec").get_parameter_value().double_value
        )
        self.poll_period = (
            self.get_parameter("poll_period").get_parameter_value().double_value
        )
        self.log_period = (
            self.get_parameter("log_period").get_parameter_value().double_value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait(self) -> int:
        deadline = time.monotonic() + self.timeout_sec
        next_log = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.poll_period)

            if self.tf_buffer.can_transform(
                self.target_frame,
                self.source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                self.get_logger().info(
                    f"Transform ready: {self.target_frame} <- {self.source_frame}"
                )
                return 0

            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"Waiting for TF {self.target_frame} <- {self.source_frame}"
                )
                next_log = time.monotonic() + self.log_period

        self.get_logger().error(
            f"Timed out waiting for TF {self.target_frame} <- "
            f"{self.source_frame} after {self.timeout_sec:.1f}s"
        )
        return 1


def main(args=None) -> int:
    rclpy.init(args=args)
    node = WaitForTransform()
    try:
        return node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
