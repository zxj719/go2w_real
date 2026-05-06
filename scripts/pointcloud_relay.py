#!/usr/bin/env python3
"""Relay PointCloud2 from a live LiDAR topic to a ROS-friendly topic.

This mirrors the working scan pipeline used by go2w_auto_explore so
the RF2O + Nav2 launch can consume the same pointcloud source.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _parameter_to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def prepare_pointcloud_for_relay(
    msg: PointCloud2,
    *,
    now_stamp,
    restamp_output: bool,
) -> PointCloud2:
    if restamp_output:
        msg.header.stamp = now_stamp
    return msg


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_relay")

        self.declare_parameter("input_topic", "/unitree/slam_lidar/points")
        self.declare_parameter("output_topic", "/cloud_relayed")
        self.declare_parameter("restamp_output", False)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.restamp_output = _parameter_to_bool(
            self.get_parameter("restamp_output").value
        )

        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, best_effort_qos)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._cb, reliable_qos
        )
        self.get_logger().info(
            f"Relaying {input_topic} (RELIABLE) -> {output_topic} (BEST_EFFORT), "
            f"restamp_output={self.restamp_output}"
        )

    def _cb(self, msg):
        self.pub.publish(
            prepare_pointcloud_for_relay(
                msg,
                now_stamp=self.get_clock().now().to_msg(),
                restamp_output=self.restamp_output,
            )
        )


def main():
    rclpy.init()
    node = PointCloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
