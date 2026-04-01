#!/usr/bin/env python3
"""
Load a waypoint YAML file, show available navigation targets, and send the
selected waypoint to Nav2's NavigateToPose action.
"""

import math
import os
import sys
import time


DEFAULT_WAYPOINT_FILE = (
    "/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml"
)
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_SERVER_TIMEOUT = 10.0


def _print_help():
    print(
        "Usage: ros2 run go2w_real navigate_to_waypoint.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        "  --waypoint-file PATH       Waypoint YAML path, default: "
        "/home/unitree/ros_ws/src/go2w_real/config/go2w_map_waypoints.yaml\n"
        "  --action-name NAME         Nav2 action name, default: /navigate_to_pose\n"
        "  --server-timeout SEC       Wait timeout for Nav2 action server, default: 10.0\n"
        "  --waypoint NAME_OR_INDEX   Send one waypoint directly without prompting\n"
        "  --list-only                Only print available waypoints and exit\n"
        "  -h, --help                 Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "waypoint_file": DEFAULT_WAYPOINT_FILE,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout": DEFAULT_SERVER_TIMEOUT,
        "waypoint_selector": None,
        "list_only": False,
    }
    ros_args = []

    i = 0
    while i < len(raw_args):
        arg = raw_args[i]

        if arg in ("-h", "--help"):
            _print_help()
            raise SystemExit(0)
        if arg == "--waypoint-file" and i + 1 < len(raw_args):
            config["waypoint_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--waypoint-file="):
            config["waypoint_file"] = os.path.expanduser(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--action-name" and i + 1 < len(raw_args):
            config["action_name"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--action-name="):
            config["action_name"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--server-timeout" and i + 1 < len(raw_args):
            config["server_timeout"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--server-timeout="):
            config["server_timeout"] = float(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--waypoint" and i + 1 < len(raw_args):
            config["waypoint_selector"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--waypoint="):
            config["waypoint_selector"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--list-only":
            config["list_only"] = True
            i += 1
            continue

        ros_args.append(arg)
        i += 1

    return config, ros_args


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def _load_waypoints(path):
    import yaml

    waypoint_path = os.path.expanduser(path)
    with open(waypoint_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    raw_waypoints = data.get("waypoints", [])
    if not isinstance(raw_waypoints, list):
        raise ValueError(f"Invalid waypoint file: 'waypoints' is not a list in {path}")

    waypoints = []
    for idx, raw_wp in enumerate(raw_waypoints, start=1):
        if not isinstance(raw_wp, dict):
            raise ValueError(
                f"Invalid waypoint entry #{idx}: expected mapping, got {type(raw_wp)}"
            )

        position = raw_wp.get("position", {}) or {}
        orientation = raw_wp.get("orientation", {}) or {}

        yaw = raw_wp.get("yaw")
        if yaw is None and all(k in orientation for k in ("x", "y", "z", "w")):
            yaw = _yaw_from_quaternion(
                float(orientation["x"]),
                float(orientation["y"]),
                float(orientation["z"]),
                float(orientation["w"]),
            )
        elif yaw is None:
            yaw = 0.0
        else:
            yaw = float(yaw)

        if all(k in orientation for k in ("x", "y", "z", "w")):
            quat = {
                "x": float(orientation["x"]),
                "y": float(orientation["y"]),
                "z": float(orientation["z"]),
                "w": float(orientation["w"]),
            }
        else:
            quat = _quaternion_from_yaw(yaw)

        waypoints.append(
            {
                "index": idx,
                "name": str(raw_wp.get("name", f"wp_{idx:02d}")),
                "frame_id": str(raw_wp.get("frame_id", "map")),
                "position": {
                    "x": float(position.get("x", 0.0)),
                    "y": float(position.get("y", 0.0)),
                    "z": float(position.get("z", 0.0)),
                },
                "yaw": yaw,
                "orientation": quat,
            }
        )

    return waypoints


def _print_waypoints(waypoints):
    print("\nAvailable waypoints:", flush=True)
    if not waypoints:
        print("  (none)", flush=True)
        return

    for wp in waypoints:
        print(
            f"  {wp['index']:>2}. {wp['name']:<16} "
            f"frame={wp['frame_id']:<6} "
            f"x={wp['position']['x']:.3f} "
            f"y={wp['position']['y']:.3f} "
            f"yaw={math.degrees(wp['yaw']):.1f} deg",
            flush=True,
        )


def _resolve_waypoint(selector, waypoints):
    text = str(selector).strip()
    if not text:
        return None

    if text.isdigit():
        index = int(text)
        for wp in waypoints:
            if wp["index"] == index:
                return wp

    lowered = text.lower()
    for wp in waypoints:
        if wp["name"].lower() == lowered:
            return wp

    return None


def _goal_status_label(status):
    from action_msgs.msg import GoalStatus

    labels = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return labels.get(status, f"STATUS_{status}")


class WaypointNavigator:
    def __init__(self, node, action_name, server_timeout):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self.node = node
        self.server_timeout = server_timeout
        self._navigate_to_pose = NavigateToPose
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._active_goal_handle = None
        self._last_feedback_log_time = 0.0

    def navigate_to(self, waypoint):
        import rclpy

        self.node.get_logger().info(
            f"Waiting for Nav2 action server for up to {self.server_timeout:.1f}s..."
        )
        if not self._client.wait_for_server(timeout_sec=self.server_timeout):
            self.node.get_logger().error("Nav2 action server is not available.")
            return False

        goal_msg = self._navigate_to_pose.Goal()
        goal_msg.pose.header.frame_id = waypoint["frame_id"]
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint["position"]["x"]
        goal_msg.pose.pose.position.y = waypoint["position"]["y"]
        goal_msg.pose.pose.position.z = waypoint["position"]["z"]
        goal_msg.pose.pose.orientation.x = waypoint["orientation"]["x"]
        goal_msg.pose.pose.orientation.y = waypoint["orientation"]["y"]
        goal_msg.pose.pose.orientation.z = waypoint["orientation"]["z"]
        goal_msg.pose.pose.orientation.w = waypoint["orientation"]["w"]

        self.node.get_logger().info(
            f"Sending Nav2 goal '{waypoint['name']}' "
            f"to x={waypoint['position']['x']:.3f}, "
            f"y={waypoint['position']['y']:.3f}, "
            f"yaw={math.degrees(waypoint['yaw']):.1f} deg"
        )

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.node.get_logger().error("Failed to send Nav2 goal.")
            return False
        if not goal_handle.accepted:
            self.node.get_logger().warn("Nav2 rejected the waypoint goal.")
            return False

        self._active_goal_handle = goal_handle
        self.node.get_logger().info("Nav2 accepted the goal, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        self._active_goal_handle = None

        if result is None:
            self.node.get_logger().error("Did not receive a Nav2 result.")
            return False

        status_name = _goal_status_label(result.status)
        if status_name == "SUCCEEDED":
            self.node.get_logger().info(
                f"Waypoint '{waypoint['name']}' reached successfully."
            )
            return True

        self.node.get_logger().warn(
            f"Waypoint '{waypoint['name']}' finished with status {status_name}."
        )
        return False

    def cancel_active_goal(self):
        import rclpy

        if self._active_goal_handle is None:
            return

        self.node.get_logger().warn("Cancelling active Nav2 goal...")
        cancel_future = self._active_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
        self._active_goal_handle = None

    def _feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self._last_feedback_log_time < 1.0:
            return

        self._last_feedback_log_time = now
        feedback = feedback_msg.feedback

        # navigation time（Foxy 有）
        nav_time = None
        if hasattr(feedback, 'navigation_time'):
            nav_time = (
                float(feedback.navigation_time.sec)
                + float(feedback.navigation_time.nanosec) * 1e-9
            )

        # distance（Foxy 有）
        distance = None
        if hasattr(feedback, 'distance_remaining'):
            distance = float(feedback.distance_remaining)

        # ETA（⚠️ Foxy 没有，这里做兼容）
        eta = None
        if hasattr(feedback, 'estimated_time_remaining'):
            eta = (
                float(feedback.estimated_time_remaining.sec)
                + float(feedback.estimated_time_remaining.nanosec) * 1e-9
            )

        # 拼接日志（自适应字段）
        log_parts = []

        if distance is not None:
            log_parts.append(f"dist={distance:.3f} m")

        if nav_time is not None:
            log_parts.append(f"time={nav_time:.1f} s")

        if eta is not None:
            log_parts.append(f"eta={eta:.1f} s")

        if log_parts:
            self.node.get_logger().info("Nav2 feedback: " + ", ".join(log_parts))
        else:
            self.node.get_logger().info("Nav2 feedback received")


def _prompt_for_waypoint(waypoints):
    while True:
        _print_waypoints(waypoints)
        try:
            raw = input(
                "\n[waypoint_nav] choose waypoint by index or name "
                "('q' to quit): "
            ).strip()
        except EOFError:
            return None

        if raw.lower() in ("q", "quit", "exit"):
            return None

        waypoint = _resolve_waypoint(raw, waypoints)
        if waypoint is not None:
            return waypoint

        print(f"[waypoint_nav] unknown waypoint '{raw}', please try again.", flush=True)


def main(args=None):
    import rclpy
    from rclpy.node import Node

    if args is None:
        raw_args = sys.argv[1:]
    else:
        raw_args = list(args)

    config, ros_args = _parse_cli_args(raw_args)
    waypoints = _load_waypoints(config["waypoint_file"])

    if not waypoints:
        print(
            f"No waypoints found in {os.path.expanduser(config['waypoint_file'])}.",
            file=sys.stderr,
        )
        raise SystemExit(1)

    if config["list_only"]:
        _print_waypoints(waypoints)
        return

    if config["waypoint_selector"] is not None:
        waypoint = _resolve_waypoint(config["waypoint_selector"], waypoints)
        if waypoint is None:
            print(
                f"Unknown waypoint '{config['waypoint_selector']}'.",
                file=sys.stderr,
            )
            _print_waypoints(waypoints)
            raise SystemExit(1)
    else:
        waypoint = None

    rclpy.init(args=ros_args)
    node = Node("navigate_to_waypoint")
    navigator = WaypointNavigator(
        node,
        config["action_name"],
        config["server_timeout"],
    )

    try:
        while rclpy.ok():
            selected = waypoint or _prompt_for_waypoint(waypoints)
            waypoint = None

            if selected is None:
                break

            navigator.navigate_to(selected)
    except KeyboardInterrupt:
        navigator.cancel_active_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
