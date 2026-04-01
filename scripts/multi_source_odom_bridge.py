#!/usr/bin/env python3
"""
Lock onto the first active official odom/pose source and republish it as /odom.

This bridge is intended for Unitree's built-in SLAM stack where the exact
odometry topic may depend on the current mode (mapping vs relocation), and
some systems expose only PoseStamped instead of Odometry.
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import tf2_ros


class MultiSourceOdomBridge(Node):
    def __init__(self):
        super().__init__("multi_source_odom_bridge")

        self.declare_parameter(
            "odom_topics",
            [
                "/unitree/slam_mapping/odom",
                "/unitree/slam_relocation/odom",
                "/utlidar/robot_odom",
                "/uslam/frontend/odom",
                "/uslam/localization/odom",
                "/lio_sam_ros2/mapping/odometry",
            ],
        )
        self.declare_parameter("pose_topics", ["/utlidar/robot_pose"])
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("selected_source_topic", "/odom_source")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("child_frame_id_override", "base")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("lock_source", True)
        self.declare_parameter("source_timeout_sec", 2.0)
        self.declare_parameter("replace_zero_stamp_with_now", True)

        self.odom_topics = list(self.get_parameter("odom_topics").value)
        self.pose_topics = list(self.get_parameter("pose_topics").value)
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.selected_source_topic = self.get_parameter("selected_source_topic").value
        self.frame_id_override = self.get_parameter("frame_id_override").value
        self.child_frame_id_override = self.get_parameter("child_frame_id_override").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.lock_source = bool(self.get_parameter("lock_source").value)
        self.source_timeout_sec = float(self.get_parameter("source_timeout_sec").value)
        self.replace_zero_stamp_with_now = bool(
            self.get_parameter("replace_zero_stamp_with_now").value
        )

        reliable_qos = QoSProfile(depth=50)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.source_pub = self.create_publisher(String, self.selected_source_topic, 10)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.source_order = []
        self.source_kind = {}
        self.source_messages = {}
        self.source_last_seen = {}
        self.selected_source = None

        for topic in self.odom_topics:
            if not topic:
                continue
            self.source_order.append(topic)
            self.source_kind[topic] = "odom"
            self.create_subscription(
                Odometry,
                topic,
                self._make_callback(topic),
                reliable_qos,
            )

        for topic in self.pose_topics:
            if not topic:
                continue
            self.source_order.append(topic)
            self.source_kind[topic] = "pose"
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_callback(topic),
                reliable_qos,
            )

        self.priority = {topic: i for i, topic in enumerate(self.source_order)}
        self.create_timer(0.1, self._selection_timer_cb)

        self.get_logger().info(
            "Watching odom sources in priority order: "
            + ", ".join(self.source_order)
        )

    def _make_callback(self, topic):
        def _callback(msg):
            self.source_messages[topic] = msg
            self.source_last_seen[topic] = time.monotonic()

            if self.selected_source == topic:
                self._publish_selected_message(topic, msg)

        return _callback

    def _selection_timer_cb(self):
        now = time.monotonic()

        if self.selected_source is not None:
            last_seen = self.source_last_seen.get(self.selected_source, 0.0)
            if now - last_seen <= self.source_timeout_sec:
                return

            self.get_logger().warn(
                f"Selected odom source timed out: {self.selected_source}. "
                "Unlocking and waiting for another active source."
            )
            self.selected_source = None

        active_topics = [
            topic
            for topic in self.source_order
            if topic in self.source_last_seen
            and now - self.source_last_seen[topic] <= self.source_timeout_sec
        ]

        if not active_topics:
            return

        best_topic = min(active_topics, key=lambda topic: self.priority[topic])
        previous_source = self.selected_source

        if previous_source is None:
            self.selected_source = best_topic
            self._announce_selected_source(best_topic, "locked onto first active source")
            self._publish_selected_message(best_topic, self.source_messages[best_topic])
            return

        if not self.lock_source and best_topic != previous_source:
            self.selected_source = best_topic
            self._announce_selected_source(best_topic, "switched to higher-priority active source")
            self._publish_selected_message(best_topic, self.source_messages[best_topic])

    def _announce_selected_source(self, topic, reason):
        msg = String()
        msg.data = topic
        self.source_pub.publish(msg)
        self.get_logger().info(f"{reason}: {topic}")

    def _publish_selected_message(self, topic, msg):
        odom = self._convert_to_odom(topic, msg)
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(tf_msg)

    def _convert_to_odom(self, topic, msg):
        odom = Odometry()

        if self.source_kind[topic] == "odom":
            odom.header = msg.header
            odom.child_frame_id = msg.child_frame_id
            odom.pose = msg.pose
            odom.twist = msg.twist
        else:
            odom.header = msg.header
            odom.child_frame_id = ""
            odom.pose.pose = msg.pose

        if self.replace_zero_stamp_with_now:
            if odom.header.stamp.sec == 0 and odom.header.stamp.nanosec == 0:
                odom.header.stamp = self.get_clock().now().to_msg()

        if self.frame_id_override:
            odom.header.frame_id = self.frame_id_override
        elif not odom.header.frame_id:
            odom.header.frame_id = "map"

        if self.child_frame_id_override:
            odom.child_frame_id = self.child_frame_id_override
        elif not odom.child_frame_id:
            odom.child_frame_id = "base"

        return odom


def main():
    rclpy.init()
    node = MultiSourceOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
