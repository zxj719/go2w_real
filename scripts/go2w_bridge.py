#!/usr/bin/env python3
"""
GO2W Bridge Node: bridges ROS2 cmd_vel to Unitree SDK2 SportClient,
and publishes odometry + TF from SportModeState.

Subscribes:
  /cmd_vel (geometry_msgs/Twist) - velocity commands from Nav2 or teleop
  /cmd_control (std_msgs/String) - mode commands: stand_up, stand_down, damp, recovery

Publishes:
  /odom (nav_msgs/Odometry) - robot odometry from SportModeState
  /tf (odom -> base) - odometry transform
  /imu/data (sensor_msgs/Imu) - IMU data from SportModeState
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf2_ros

from unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert euler angles to geometry_msgs Quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Go2wBridge(Node):
    def __init__(self):
        super().__init__("go2w_bridge")

        # Parameters
        self.declare_parameter("network_interface", "eth0")
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("publish_odom_tf", True)

        net_iface = self.get_parameter("network_interface").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value

        # State
        self.last_cmd_vel_time = time.time()
        self.latest_twist = Twist()
        self.lock = threading.Lock()

        # Initialize Unitree SDK2
        self.get_logger().info(f"Initializing Unitree SDK on interface: {net_iface}")
        ChannelFactoryInitialize(0, net_iface)

        # SportClient for sending commands
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.get_logger().info("SportClient initialized")

        # Subscribe to SportModeState for odometry
        self.sport_state = None
        self.sport_state_lock = threading.Lock()
        self.state_sub = ChannelSubscriber(
            "rt/sportmodestate", SportModeState_
        )
        self.state_sub.Init(self._sport_state_callback, 10)

        # Callback groups
        cmd_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        odom_cb_group = MutuallyExclusiveCallbackGroup()

        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self._cmd_vel_callback,
            10,
            callback_group=cmd_cb_group,
        )
        self.cmd_control_sub = self.create_subscription(
            String,
            "cmd_control",
            self._cmd_control_callback,
            10,
            callback_group=cmd_cb_group,
        )

        # ROS2 publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timers
        # Send velocity to robot at 20 Hz
        self.cmd_timer = self.create_timer(
            0.05, self._cmd_timer_callback, callback_group=timer_cb_group
        )
        # Publish odom at 50 Hz
        self.odom_timer = self.create_timer(
            0.02, self._odom_timer_callback, callback_group=odom_cb_group
        )

        self.get_logger().info("GO2W Bridge ready. Waiting for cmd_vel...")

    def _sport_state_callback(self, msg: SportModeState_):
        """SDK callback (runs in SDK thread). Just store the latest state."""
        with self.sport_state_lock:
            self.sport_state = msg

    def _cmd_vel_callback(self, msg: Twist):
        """ROS2 cmd_vel subscriber callback."""
        with self.lock:
            self.latest_twist = msg
            self.last_cmd_vel_time = time.time()

    def _cmd_control_callback(self, msg: String):
        """Handle mode-switching commands."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Control command: {cmd}")

        if cmd == "stand_up":
            self.sport_client.StandUp()
        elif cmd == "stand_down":
            self.sport_client.StandDown()
        elif cmd == "damp":
            self.sport_client.Damp()
        elif cmd == "recovery":
            self.sport_client.RecoveryStand()
        elif cmd == "balance":
            self.sport_client.BalanceStand()
        elif cmd == "stop":
            self.sport_client.StopMove()
        else:
            self.get_logger().warn(f"Unknown control command: {cmd}")

    def _cmd_timer_callback(self):
        """Send velocity command to robot at fixed rate."""
        with self.lock:
            twist = self.latest_twist
            elapsed = time.time() - self.last_cmd_vel_time

        # Safety: stop if no cmd_vel received within timeout
        if elapsed > self.cmd_vel_timeout:
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            vx = twist.linear.x
            vy = twist.linear.y
            vyaw = twist.angular.z

        self.sport_client.Move(vx, vy, vyaw)

    def _odom_timer_callback(self):
        """Publish odometry from SportModeState."""
        with self.sport_state_lock:
            state = self.sport_state

        if state is None:
            return

        now = self.get_clock().now().to_msg()

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position from state (state.position is [x, y, z])
        odom.pose.pose.position.x = float(state.position[0])
        odom.pose.pose.position.y = float(state.position[1])
        odom.pose.pose.position.z = float(state.position[2])

        # Orientation from IMU state (state.imu_state.rpy is [roll, pitch, yaw])
        roll = float(state.imu_state.rpy[0])
        pitch = float(state.imu_state.rpy[1])
        yaw = float(state.imu_state.rpy[2])
        odom.pose.pose.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Velocity (state.velocity is [vx, vy, vz] in body frame)
        odom.twist.twist.linear.x = float(state.velocity[0])
        odom.twist.twist.linear.y = float(state.velocity[1])
        odom.twist.twist.linear.z = float(state.velocity[2])
        odom.twist.twist.angular.z = float(state.yaw_speed)

        self.odom_pub.publish(odom)

        # Publish TF: odom -> base
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # Publish IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = euler_to_quaternion(roll, pitch, yaw)
        # Quaternion from IMU (state.imu_state.quaternion is [w, x, y, z])
        imu_msg.angular_velocity.x = float(state.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(state.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(state.imu_state.gyroscope[2])
        imu_msg.linear_acceleration.x = float(state.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(state.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(state.imu_state.accelerometer[2])
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2wBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        node.get_logger().info("Shutting down, stopping robot...")
        node.sport_client.StopMove()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
