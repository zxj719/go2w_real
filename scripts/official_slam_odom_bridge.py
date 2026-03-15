#!/usr/bin/env python3
"""
Bridge official Unitree SLAM odometry topic to Nav2-friendly odom + TF.

Input:
  /unitree/slam_mapping/odom (nav_msgs/Odometry)

Output:
  /odom (nav_msgs/Odometry)
  tf: map -> <child_frame_id>
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OfficialSlamOdomBridge(Node):
    def __init__(self):
        super().__init__("official_slam_odom_bridge")

        self.declare_parameter("input_odom_topic", "/unitree/slam_mapping/odom")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("child_frame_id_override", "lidar")
        self.declare_parameter("publish_tf", True)

        input_topic = self.get_parameter("input_odom_topic").value
        output_topic = self.get_parameter("output_odom_topic").value
        self.frame_id_override = self.get_parameter("frame_id_override").value
        self.child_frame_override = self.get_parameter("child_frame_id_override").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(Odometry, input_topic, self._on_odom, qos)
        self.odom_pub = self.create_publisher(Odometry, output_topic, 20)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f"Official SLAM odom bridge started: {input_topic} -> {output_topic}, "
            f"frame_override='{self.frame_id_override}', child_override='{self.child_frame_override}'"
        )

    def _on_odom(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override
        if self.child_frame_override:
            out.child_frame_id = self.child_frame_override

        self.odom_pub.publish(out)

        if not self.publish_tf:
            return

        t = TransformStamped()
        t.header = out.header
        t.child_frame_id = out.child_frame_id
        t.transform.translation.x = out.pose.pose.position.x
        t.transform.translation.y = out.pose.pose.position.y
        t.transform.translation.z = out.pose.pose.position.z
        t.transform.rotation = out.pose.pose.orientation
        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = OfficialSlamOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
