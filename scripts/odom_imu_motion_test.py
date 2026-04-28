#!/usr/bin/env python3
"""Controlled odom/IMU motion test for short GO2W localization validation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


@dataclass(frozen=True)
class MotionSegment:
    name: str
    duration_sec: float
    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def is_hold(self) -> bool:
        return abs(self.linear_x) < 1.0e-9 and abs(self.angular_z) < 1.0e-9


@dataclass
class OdomSample:
    time_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class ImuSample:
    time_sec: float
    angular_z: float


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def validate_motion_limits(
    *,
    linear_distance_m: float,
    linear_speed_mps: float,
    turn_angle_deg: float,
    angular_speed_radps: float,
) -> None:
    if not 0.0 < linear_distance_m <= 0.5:
        raise ValueError("linear distance must be in (0.0, 0.5] m")
    if linear_speed_mps <= 0.1:
        raise ValueError("linear speed must be greater than 0.1 m/s")
    if turn_angle_deg <= 0.0 or turn_angle_deg > 90.0:
        raise ValueError("turn angle must be in (0.0, 90.0] degrees")
    if angular_speed_radps <= 0.1:
        raise ValueError("angular speed must be greater than 0.1 rad/s")


def build_motion_plan(
    *,
    linear_distance_m: float,
    linear_speed_mps: float,
    turn_angle_deg: float,
    angular_speed_radps: float,
    settle_sec: float,
    initial_hold_sec: float,
    final_hold_sec: float,
) -> List[MotionSegment]:
    validate_motion_limits(
        linear_distance_m=linear_distance_m,
        linear_speed_mps=linear_speed_mps,
        turn_angle_deg=turn_angle_deg,
        angular_speed_radps=angular_speed_radps,
    )
    turn_duration_sec = math.radians(turn_angle_deg) / angular_speed_radps
    linear_duration_sec = linear_distance_m / linear_speed_mps
    distance_label = f"{linear_distance_m:.2f}m"
    angle_label = f"{turn_angle_deg:.0f}deg"

    return [
        MotionSegment("initial_hold", initial_hold_sec),
        MotionSegment(
            f"turn_left_{angle_label}",
            turn_duration_sec,
            angular_z=angular_speed_radps,
        ),
        MotionSegment("settle_after_left", settle_sec),
        MotionSegment(
            f"turn_right_{angle_label}",
            turn_duration_sec,
            angular_z=-angular_speed_radps,
        ),
        MotionSegment("settle_after_turns", settle_sec),
        MotionSegment(
            f"forward_{distance_label}",
            linear_duration_sec,
            linear_x=linear_speed_mps,
        ),
        MotionSegment("settle_after_forward", settle_sec),
        MotionSegment(
            f"reverse_{distance_label}",
            linear_duration_sec,
            linear_x=-linear_speed_mps,
        ),
        MotionSegment("final_hold", final_hold_sec),
    ]


def format_plan(plan: List[MotionSegment]) -> str:
    lines = ["Planned odom/IMU motion test:"]
    for segment in plan:
        lines.append(
            f"  {segment.name:22s} duration={segment.duration_sec:5.2f}s "
            f"vx={segment.linear_x: .3f} m/s wz={segment.angular_z: .3f} rad/s"
        )
    lines.append("Dry-run only. Re-run with --execute to publish /cmd_vel.")
    return "\n".join(lines)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a short, bounded motion profile for checking odom and IMU "
            "response before enabling scan-degradation gating."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually publish cmd_vel",
    )
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--sport-odom-topic", default="/sport_odom")
    parser.add_argument(
        "--imu-topics",
        default="/sport_imu,/lowstate_imu,/utlidar_imu_base",
        help="comma-separated IMU topics to summarize",
    )
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--linear-distance-m", type=float, default=0.5)
    parser.add_argument("--linear-speed-mps", type=float, default=0.15)
    parser.add_argument("--turn-angle-deg", type=float, default=90.0)
    parser.add_argument("--angular-speed-radps", type=float, default=0.35)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--initial-hold-sec", type=float, default=2.0)
    parser.add_argument("--final-hold-sec", type=float, default=3.0)
    parser.add_argument(
        "--max-observed-xy-m",
        type=float,
        default=0.75,
        help="abort if any observed odom displacement from test start exceeds this value",
    )
    return parser.parse_args(argv)


class MotionTestNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("odom_imu_motion_test")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.odom_samples: Dict[str, List[OdomSample]] = {
            args.odom_topic: [],
            args.sport_odom_topic: [],
        }
        self.imu_samples: Dict[str, List[ImuSample]] = {
            topic.strip(): []
            for topic in args.imu_topics.split(",")
            if topic.strip()
        }

        for topic in self.odom_samples:
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self._odom_callback(topic, msg),
                20,
            )
        for topic in self.imu_samples:
            self.create_subscription(
                Imu,
                topic,
                lambda msg, topic=topic: self._imu_callback(topic, msg),
                50,
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _odom_callback(self, topic: str, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.odom_samples[topic].append(
            OdomSample(
                time_sec=self._now_sec(),
                x=position.x,
                y=position.y,
                yaw=yaw_from_quaternion(msg.pose.pose.orientation),
            )
        )

    def _imu_callback(self, topic: str, msg: Imu) -> None:
        self.imu_samples[topic].append(
            ImuSample(
                time_sec=self._now_sec(),
                angular_z=msg.angular_velocity.z,
            )
        )

    def publish_velocity(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)

    def publish_stop(self, hold_sec: float = 0.0) -> None:
        deadline = self._now_sec() + max(hold_sec, 0.0)
        while rclpy.ok() and self._now_sec() <= deadline:
            self.publish_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._sleep_rate()
        self.publish_velocity(0.0, 0.0)

    def _sleep_rate(self) -> None:
        rclpy.spin_once(self, timeout_sec=max(0.001, 1.0 / self.args.rate_hz))

    def _observed_displacement_exceeded(self) -> bool:
        for samples in self.odom_samples.values():
            if len(samples) < 2:
                continue
            start = samples[0]
            current = samples[-1]
            if (
                math.hypot(current.x - start.x, current.y - start.y)
                > self.args.max_observed_xy_m
            ):
                return True
        return False

    def run_plan(self, plan: List[MotionSegment]) -> int:
        self.get_logger().info(format_plan(plan))
        for segment in plan:
            before_odom = {
                topic: len(samples) for topic, samples in self.odom_samples.items()
            }
            before_imu = {
                topic: len(samples) for topic, samples in self.imu_samples.items()
            }
            deadline = self._now_sec() + segment.duration_sec
            self.get_logger().info(f"segment start: {segment.name}")
            while rclpy.ok() and self._now_sec() < deadline:
                self.publish_velocity(segment.linear_x, segment.angular_z)
                if self._observed_displacement_exceeded():
                    self.get_logger().error(
                        "observed odom displacement exceeded safety limit; stopping"
                    )
                    self.publish_stop(1.0)
                    return 2
                self._sleep_rate()
            self.publish_stop(0.2)
            self._log_segment_summary(segment, before_odom, before_imu)
        self.publish_stop(1.0)
        return 0

    def _log_segment_summary(
        self,
        segment: MotionSegment,
        before_odom: Dict[str, int],
        before_imu: Dict[str, int],
    ) -> None:
        parts = [f"segment done: {segment.name}"]
        for topic, samples in self.odom_samples.items():
            segment_samples = samples[before_odom[topic] :]
            if len(segment_samples) < 2:
                continue
            start = segment_samples[0]
            end = segment_samples[-1]
            dxy = math.hypot(end.x - start.x, end.y - start.y)
            dyaw_deg = math.degrees(shortest_angle_delta(start.yaw, end.yaw))
            parts.append(f"{topic}: dxy={dxy:.3f}m dyaw={dyaw_deg:.1f}deg")
        for topic, samples in self.imu_samples.items():
            segment_samples = samples[before_imu[topic] :]
            if len(segment_samples) < 2:
                continue
            yaw_rad = 0.0
            for previous, current in zip(segment_samples, segment_samples[1:]):
                dt = max(current.time_sec - previous.time_sec, 0.0)
                yaw_rad += 0.5 * (previous.angular_z + current.angular_z) * dt
            parts.append(f"{topic}: gyro_yaw={math.degrees(yaw_rad):.1f}deg")
        self.get_logger().info(" | ".join(parts))


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    plan = build_motion_plan(
        linear_distance_m=args.linear_distance_m,
        linear_speed_mps=args.linear_speed_mps,
        turn_angle_deg=args.turn_angle_deg,
        angular_speed_radps=args.angular_speed_radps,
        settle_sec=args.settle_sec,
        initial_hold_sec=args.initial_hold_sec,
        final_hold_sec=args.final_hold_sec,
    )
    if not args.execute:
        print(format_plan(plan))
        return 0

    rclpy.init(args=None)
    node = MotionTestNode(args)
    try:
        return node.run_plan(plan)
    except KeyboardInterrupt:
        node.publish_stop(1.0)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
