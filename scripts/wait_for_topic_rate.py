#!/usr/bin/env python3
"""Wait until a ROS 2 topic is publishing at a minimum receive rate."""

from __future__ import annotations

from collections import deque
import sys
from typing import Deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class TopicRateWaiter(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_topic_rate")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("topic_type", "sensor_msgs/msg/LaserScan")
        self.declare_parameter("min_rate_hz", 4.0)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("window_sec", 2.0)
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("log_period", 2.0)

        self.topic = str(self.get_parameter("topic").value)
        topic_type = str(self.get_parameter("topic_type").value)
        self.min_rate_hz = float(self.get_parameter("min_rate_hz").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.log_period = float(self.get_parameter("log_period").value)

        if self.min_samples < 2:
            self.min_samples = 2
        if self.window_sec <= 0.0:
            self.window_sec = 1.0

        try:
            message_type = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            raise RuntimeError(f"unsupported topic_type '{topic_type}': {exc}") from exc

        self.samples: Deque[float] = deque()
        self.start_sec = self._now_sec()
        self.last_log_sec = self.start_sec - self.log_period
        self.ready = False

        self.create_subscription(
            message_type,
            self.topic,
            self._message_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Waiting for {self.topic} >= {self.min_rate_hz:.2f} Hz "
            f"with {self.min_samples} samples over {self.window_sec:.1f}s"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _message_callback(self, _msg: object) -> None:
        now_sec = self._now_sec()
        self.samples.append(now_sec)
        while self.samples and now_sec - self.samples[0] > self.window_sec:
            self.samples.popleft()

        rate_hz = self._current_rate_hz()
        if len(self.samples) >= self.min_samples and rate_hz >= self.min_rate_hz:
            self.ready = True

    def _current_rate_hz(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        duration = self.samples[-1] - self.samples[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.samples) - 1) / duration

    def timed_out(self) -> bool:
        return (
            self.timeout_sec > 0.0
            and self._now_sec() - self.start_sec > self.timeout_sec
        )

    def maybe_log_status(self) -> None:
        now_sec = self._now_sec()
        if now_sec - self.last_log_sec < self.log_period:
            return
        self.last_log_sec = now_sec
        self.get_logger().info(
            f"Still waiting for {self.topic}: samples={len(self.samples)}, "
            f"rate={self._current_rate_hz():.2f} Hz"
        )


def main() -> int:
    rclpy.init()
    node = TopicRateWaiter()
    try:
        while rclpy.ok() and not node.ready and not node.timed_out():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.maybe_log_status()

        if node.ready:
            node.get_logger().info(
                f"{node.topic} ready: rate={node._current_rate_hz():.2f} Hz, "
                f"samples={len(node.samples)}"
            )
            return 0

        node.get_logger().error(
            f"Timed out waiting for {node.topic}: "
            f"rate={node._current_rate_hz():.2f} Hz, samples={len(node.samples)}"
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
