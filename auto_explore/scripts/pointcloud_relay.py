#!/usr/bin/env python3
"""Relay PointCloud2 from RELIABLE publisher to RELIABLE republish.

xt16_driver publishes /unitree/slam_lidar/points with RELIABLE QoS.
pointcloud_to_laserscan subscribes with BEST_EFFORT (SensorDataQoS).
BEST_EFFORT subscriber cannot receive from RELIABLE publisher in CycloneDDS.

This relay subscribes RELIABLE and republishes with BEST_EFFORT so
pointcloud_to_laserscan can receive the data.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_relay")

        self.declare_parameter("input_topic", "/unitree/slam_lidar/points")
        self.declare_parameter("output_topic", "/cloud_relayed")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

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
            f"Relaying {input_topic} (RELIABLE) -> {output_topic} (BEST_EFFORT)"
        )

    def _cb(self, msg):
        self.pub.publish(msg)


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
