#!/usr/bin/env python3
"""
Load a waypoint YAML file, show available navigation targets, and send the
selected waypoint to Nav2's NavigateToPose action.
"""

import math
import os
import sys
import time


def _get_env_text(name, default):
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return os.path.expanduser(value)


DEFAULT_WAYPOINT_FILE = _get_env_text(
    "GO2W_WAYPOINT_FILE",
    "/home/unitree/ros_ws/src/go2w_real/config/go2w_map_waypoints.yaml",#go2w_waypoints  go2w_map_waypoints
)
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_PLANNER_ACTION_NAME = "/compute_path_to_pose"
DEFAULT_SERVER_TIMEOUT = 10.0
DEFAULT_ABORT_REPLAN_RETRIES = 1
DEFAULT_IGNORE_WAYPOINT_YAW = True
DEFAULT_NEAR_GOAL_DISTANCE = 0.40
DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base"
DEFAULT_SUCCESS_CHECK_DISTANCE = 0.75
DEFAULT_SUCCESS_CHECK_TIMEOUT = 1.0
DEFAULT_BATCH_INTER_GOAL_DELAY = 0.6


def _print_help():
    print(
        f"Usage: ros2 run go2w_real navigate_to_waypoint.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        f"  --waypoint-file PATH       Waypoint YAML path, default: {DEFAULT_WAYPOINT_FILE}\n"
        f"  --action-name NAME         Nav2 action name, default: {DEFAULT_ACTION_NAME}\n"
        f"  --server-timeout SEC       Wait timeout for Nav2 action server, default: {DEFAULT_SERVER_TIMEOUT:.1f}\n"
        f"  --abort-replan-retries N   Retry after Nav2 abort if planner still has a path, default: {DEFAULT_ABORT_REPLAN_RETRIES}\n"
        f"  --near-goal-distance M     Treat this CLI goal as reached when feedback distance is within M, default: {DEFAULT_NEAR_GOAL_DISTANCE:.2f}\n"
        f"  --success-check-distance M Doublecheck final TF pose is within M of target, default: {DEFAULT_SUCCESS_CHECK_DISTANCE:.2f}\n"
        f"  --success-check-timeout S  Keep sampling TF for up to S seconds during doublecheck, default: {DEFAULT_SUCCESS_CHECK_TIMEOUT:.1f}\n"
        f"  --map-frame FRAME          Map frame used by the success doublecheck, default: {DEFAULT_MAP_FRAME}\n"
        f"  --base-frame FRAME         Base frame used by the success doublecheck, default: {DEFAULT_BASE_FRAME}\n"
        f"  --batch-inter-goal-delay S Wait S seconds between batch waypoints, default: {DEFAULT_BATCH_INTER_GOAL_DELAY:.1f}\n"
        "  --use-waypoint-yaw        Respect stored waypoint yaw instead of stopping on XY only\n"
        "  --waypoint ID_NAME_OR_INDEX Send one waypoint directly without prompting\n"
        "  --waypoint-sequence CSV   Send multiple waypoints in order, e.g. POI_019,POI_020\n"
        "  --all-waypoints           Traverse every waypoint in file order without prompting\n"
        "  --continue-on-failure     In batch mode, continue after failures instead of stopping immediately\n"
        "  --list-only                Only print available waypoints and exit\n"
        "  -h, --help                 Show this help message\n"
        "\n"
        "Environment overrides:\n"
        "  GO2W_WAYPOINT_FILE\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "waypoint_file": DEFAULT_WAYPOINT_FILE,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout": DEFAULT_SERVER_TIMEOUT,
        "abort_replan_retries": DEFAULT_ABORT_REPLAN_RETRIES,
        "near_goal_distance": DEFAULT_NEAR_GOAL_DISTANCE,
        "success_check_distance": DEFAULT_SUCCESS_CHECK_DISTANCE,
        "success_check_timeout": DEFAULT_SUCCESS_CHECK_TIMEOUT,
        "map_frame": DEFAULT_MAP_FRAME,
        "base_frame": DEFAULT_BASE_FRAME,
        "batch_inter_goal_delay": DEFAULT_BATCH_INTER_GOAL_DELAY,
        "ignore_waypoint_yaw": DEFAULT_IGNORE_WAYPOINT_YAW,
        "waypoint_selector": None,
        "waypoint_sequence": None,
        "all_waypoints": False,
        "continue_on_failure": False,
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
        if arg == "--abort-replan-retries" and i + 1 < len(raw_args):
            config["abort_replan_retries"] = max(0, int(raw_args[i + 1]))
            i += 2
            continue
        if arg.startswith("--abort-replan-retries="):
            config["abort_replan_retries"] = max(0, int(arg.split("=", 1)[1]))
            i += 1
            continue
        if arg == "--near-goal-distance" and i + 1 < len(raw_args):
            config["near_goal_distance"] = max(0.0, float(raw_args[i + 1]))
            i += 2
            continue
        if arg.startswith("--near-goal-distance="):
            config["near_goal_distance"] = max(0.0, float(arg.split("=", 1)[1]))
            i += 1
            continue
        if arg == "--success-check-distance" and i + 1 < len(raw_args):
            config["success_check_distance"] = max(0.0, float(raw_args[i + 1]))
            i += 2
            continue
        if arg.startswith("--success-check-distance="):
            config["success_check_distance"] = max(
                0.0, float(arg.split("=", 1)[1])
            )
            i += 1
            continue
        if arg == "--success-check-timeout" and i + 1 < len(raw_args):
            config["success_check_timeout"] = max(0.0, float(raw_args[i + 1]))
            i += 2
            continue
        if arg.startswith("--success-check-timeout="):
            config["success_check_timeout"] = max(
                0.0, float(arg.split("=", 1)[1])
            )
            i += 1
            continue
        if arg == "--map-frame" and i + 1 < len(raw_args):
            config["map_frame"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--map-frame="):
            config["map_frame"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--base-frame" and i + 1 < len(raw_args):
            config["base_frame"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--base-frame="):
            config["base_frame"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--batch-inter-goal-delay" and i + 1 < len(raw_args):
            config["batch_inter_goal_delay"] = max(0.0, float(raw_args[i + 1]))
            i += 2
            continue
        if arg.startswith("--batch-inter-goal-delay="):
            config["batch_inter_goal_delay"] = max(
                0.0, float(arg.split("=", 1)[1])
            )
            i += 1
            continue
        if arg == "--use-waypoint-yaw":
            config["ignore_waypoint_yaw"] = False
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
        if arg == "--waypoint-sequence" and i + 1 < len(raw_args):
            config["waypoint_sequence"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--waypoint-sequence="):
            config["waypoint_sequence"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--all-waypoints":
            config["all_waypoints"] = True
            i += 1
            continue
        if arg == "--continue-on-failure":
            config["continue_on_failure"] = True
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
                "id": str(raw_wp.get("id", raw_wp.get("name", f"wp_{idx:02d}"))),
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
            f"  {wp['index']:>2}. id={wp['id']:<12} "
            f"name={wp['name']:<16} "
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
        if wp["id"].lower() == lowered:
            return wp

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


def _xy_distance_to_waypoint(waypoint, pose_xy):
    dx = float(waypoint["position"]["x"]) - float(pose_xy["x"])
    dy = float(waypoint["position"]["y"]) - float(pose_xy["y"])
    return math.hypot(dx, dy)


def _parse_waypoint_sequence(text):
    return [item.strip() for item in str(text).split(",") if item.strip()]


def _resolve_waypoint_batch(config, waypoints):
    if config["all_waypoints"]:
        return list(waypoints)

    if config["waypoint_sequence"] is None:
        return None

    selected = []
    unknown = []
    for selector in _parse_waypoint_sequence(config["waypoint_sequence"]):
        waypoint = _resolve_waypoint(selector, waypoints)
        if waypoint is None:
            unknown.append(selector)
        else:
            selected.append(waypoint)

    if unknown:
        raise ValueError(
            "Unknown waypoint(s) in sequence: " + ", ".join(unknown)
        )

    return selected


def _spin_delay(node, delay_sec):
    import rclpy

    remaining = max(0.0, float(delay_sec))
    if remaining <= 0.0:
        return

    deadline = time.monotonic() + remaining
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))


def _apply_waypoint_to_pose_stamped(
    node,
    pose_stamped,
    waypoint,
    ignore_waypoint_yaw=DEFAULT_IGNORE_WAYPOINT_YAW,
):
    pose_stamped.header.frame_id = waypoint["frame_id"]
    pose_stamped.header.stamp = node.get_clock().now().to_msg()
    pose_stamped.pose.position.x = waypoint["position"]["x"]
    pose_stamped.pose.position.y = waypoint["position"]["y"]
    pose_stamped.pose.position.z = waypoint["position"]["z"]
    if ignore_waypoint_yaw:
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
    else:
        pose_stamped.pose.orientation.x = waypoint["orientation"]["x"]
        pose_stamped.pose.orientation.y = waypoint["orientation"]["y"]
        pose_stamped.pose.orientation.z = waypoint["orientation"]["z"]
        pose_stamped.pose.orientation.w = waypoint["orientation"]["w"]


class WaypointNavigator:
    def __init__(
        self,
        node,
        action_name,
        server_timeout,
        abort_replan_retries,
        near_goal_distance=DEFAULT_NEAR_GOAL_DISTANCE,
        ignore_waypoint_yaw=DEFAULT_IGNORE_WAYPOINT_YAW,
        map_frame=DEFAULT_MAP_FRAME,
        base_frame=DEFAULT_BASE_FRAME,
        success_check_distance=DEFAULT_SUCCESS_CHECK_DISTANCE,
        success_check_timeout=DEFAULT_SUCCESS_CHECK_TIMEOUT,
    ):
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        import tf2_ros

        self.node = node
        self.server_timeout = server_timeout
        self.abort_replan_retries = max(0, int(abort_replan_retries))
        self.near_goal_distance = max(0.0, float(near_goal_distance))
        self.ignore_waypoint_yaw = bool(ignore_waypoint_yaw)
        self.map_frame = str(map_frame)
        self.base_frame = str(base_frame)
        self.success_check_distance = max(0.0, float(success_check_distance))
        self.success_check_timeout = max(0.0, float(success_check_timeout))
        self._compute_path_to_pose = ComputePathToPose
        self._navigate_to_pose = NavigateToPose
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._planner_client = ActionClient(
            node,
            ComputePathToPose,
            DEFAULT_PLANNER_ACTION_NAME,
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self.node,
            spin_thread=False,
        )
        self._active_goal_handle = None
        self._last_feedback_log_time = 0.0
        self._near_goal_seen = False
        self._near_goal_cancel_future = None
        self._last_feedback_pose = None

    def navigate_to(self, waypoint):
        retry_budget = self.abort_replan_retries

        for attempt_index in range(retry_budget + 1):
            status_name = self._navigate_once(waypoint)
            if status_name == "SUCCEEDED":
                return True
            if status_name == "DOUBLECHECK_FAILED":
                if attempt_index >= retry_budget:
                    self.node.get_logger().error(
                        "Waypoint "
                        f"'{waypoint['name']}' reported success from Nav2 but failed the final TF doublecheck."
                    )
                    return False

                try:
                    has_path = self.planner_has_path(waypoint)
                except Exception as exc:
                    self.node.get_logger().error(
                        "Nav2 reported success before the robot reached the target, "
                        f"and replanning probe failed: {exc}"
                    )
                    return False

                if not has_path:
                    self.node.get_logger().error(
                        "Waypoint "
                        f"'{waypoint['name']}' reported success from Nav2 but failed the final TF doublecheck."
                    )
                    return False

                next_attempt = attempt_index + 1
                self.node.get_logger().warn(
                    "Waypoint "
                    f"'{waypoint['name']}' reported success before the robot reached the target; "
                    "planner still has a path, retrying current goal "
                    f"(attempt {next_attempt}/{retry_budget})."
                )
                _spin_delay(self.node, DEFAULT_BATCH_INTER_GOAL_DELAY)
                continue
            if status_name != "ABORTED":
                return False

            if attempt_index >= retry_budget:
                self.node.get_logger().error(
                    f"Waypoint '{waypoint['name']}' aborted and retry budget exhausted."
                )
                return False

            try:
                has_path = self.planner_has_path(waypoint)
            except Exception as exc:
                self.node.get_logger().error(
                    "Nav2 aborted and replanning probe failed: "
                    f"{exc}"
                )
                return False

            if not has_path:
                self.node.get_logger().error(
                    f"Waypoint '{waypoint['name']}' aborted and planner could not find a path to the target."
                )
                return False

            next_attempt = attempt_index + 1
            self.node.get_logger().info(
                "Planner still has a path; retrying current goal "
                f"(attempt {next_attempt}/{retry_budget})."
            )

        return False

    def _navigate_once(self, waypoint):
        import rclpy

        self._near_goal_seen = False
        self._near_goal_cancel_future = None
        self.node.get_logger().info(
            f"Waiting for Nav2 action server for up to {self.server_timeout:.1f}s..."
        )
        if not self._client.wait_for_server(timeout_sec=self.server_timeout):
            self.node.get_logger().error("Nav2 action server is not available.")
            return "SERVER_UNAVAILABLE"

        goal_msg = self._navigate_to_pose.Goal()
        _apply_waypoint_to_pose_stamped(
            self.node,
            goal_msg.pose,
            waypoint,
            self.ignore_waypoint_yaw,
        )

        yaw_text = (
            "yaw=ignored"
            if self.ignore_waypoint_yaw
            else f"yaw={math.degrees(waypoint['yaw']):.1f} deg"
        )
        self.node.get_logger().info(
            f"Sending Nav2 goal '{waypoint['name']}' "
            f"to x={waypoint['position']['x']:.3f}, "
            f"y={waypoint['position']['y']:.3f}, "
            f"{yaw_text}"
        )

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.node.get_logger().error("Failed to send Nav2 goal.")
            return "SEND_GOAL_FAILED"
        if not goal_handle.accepted:
            self.node.get_logger().warn("Nav2 rejected the waypoint goal.")
            return "REJECTED"

        self._active_goal_handle = goal_handle
        self.node.get_logger().info("Nav2 accepted the goal, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        self._active_goal_handle = None

        if result is None:
            self.node.get_logger().error("Did not receive a Nav2 result.")
            return "NO_RESULT"

        status_name = _goal_status_label(result.status)
        if status_name == "SUCCEEDED" or self._should_accept_near_goal_status(
            status_name
        ):
            if not self._doublecheck_waypoint_reached(waypoint):
                return "DOUBLECHECK_FAILED"
            self.node.get_logger().info(
                f"Waypoint '{waypoint['name']}' reached successfully."
            )
            return "SUCCEEDED"

        self.node.get_logger().warn(
            f"Waypoint '{waypoint['name']}' finished with status {status_name}."
        )
        return status_name

    def planner_has_path(self, waypoint):
        import rclpy

        if not self._planner_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                "ComputePathToPose action server is not available within "
                f"{self.server_timeout:.1f}s"
            )

        goal_msg = self._compute_path_to_pose.Goal()
        _apply_waypoint_to_pose_stamped(
            self.node,
            goal_msg.pose,
            waypoint,
            self.ignore_waypoint_yaw,
        )
        goal_msg.planner_id = "GridBased"

        send_goal_future = self._planner_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        if result is None or _goal_status_label(result.status) != "SUCCEEDED":
            return False

        path = getattr(result.result, "path", None)
        poses = getattr(path, "poses", None)
        return bool(poses)

    def cancel_active_goal(self):
        import rclpy

        if self._active_goal_handle is None:
            return

        self.node.get_logger().warn("Cancelling active Nav2 goal...")
        cancel_future = self._active_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
        self._active_goal_handle = None

    def destroy(self):
        for client_name in ("_planner_client", "_client"):
            client = getattr(self, client_name, None)
            if client is None:
                continue
            try:
                client.destroy()
            except Exception:
                pass
            setattr(self, client_name, None)
        self._tf_listener = None
        self._tf_buffer = None

    def _should_accept_near_goal_status(self, status_name):
        if not self.ignore_waypoint_yaw:
            return False
        if self.near_goal_distance <= 0.0:
            return False
        if not self._near_goal_seen:
            return False
        return status_name in ("ABORTED", "CANCELED")

    def _maybe_request_near_goal_cancel(self, distance):
        if not self.ignore_waypoint_yaw:
            return
        if self.near_goal_distance <= 0.0:
            return
        if distance is None or distance > self.near_goal_distance:
            return

        self._near_goal_seen = True
        if self._active_goal_handle is None or self._near_goal_cancel_future is not None:
            return

        self.node.get_logger().info(
            "Near-goal threshold reached; cancelling Nav2 goal and accepting this waypoint."
        )
        self._near_goal_cancel_future = self._active_goal_handle.cancel_goal_async()

    def _feedback_callback(self, feedback_msg):
        now = time.monotonic()
        feedback = feedback_msg.feedback

        current_pose = getattr(feedback, "current_pose", None)
        if current_pose is not None:
            position = current_pose.pose.position
            self._last_feedback_pose = {
                "frame_id": str(current_pose.header.frame_id),
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
            }

        if now - self._last_feedback_log_time < 1.0:
            return

        self._last_feedback_log_time = now

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
            self._maybe_request_near_goal_cancel(distance)

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

    def _lookup_current_pose(self):
        import rclpy
        from rclpy.duration import Duration

        transform = self._tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.0),
        )
        translation = transform.transform.translation
        return {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
        }

    def _lookup_feedback_pose(self):
        pose = self._last_feedback_pose
        if pose is None:
            return None
        if str(pose.get("frame_id", "")) != self.map_frame:
            return None
        return pose

    def _can_lookup_current_pose(self):
        import rclpy
        from rclpy.duration import Duration

        return self._tf_buffer.can_transform(
            self.map_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.0),
        )

    def _doublecheck_waypoint_reached(self, waypoint):
        import rclpy

        if self.success_check_distance <= 0.0:
            return True

        best_distance = None
        last_error = None
        deadline = time.monotonic() + self.success_check_timeout

        while True:
            try:
                feedback_pose = self._lookup_feedback_pose()
                if feedback_pose is not None:
                    feedback_distance = _xy_distance_to_waypoint(
                        waypoint,
                        feedback_pose,
                    )
                    if best_distance is None or feedback_distance < best_distance:
                        best_distance = feedback_distance

                    if feedback_distance <= self.success_check_distance:
                        self.node.get_logger().info(
                            "Success doublecheck passed for "
                            f"'{waypoint['name']}': final_xy_error={feedback_distance:.3f} m "
                            f"(threshold={self.success_check_distance:.3f} m), "
                            "using Nav2 feedback pose."
                        )
                        return True

                if self._can_lookup_current_pose():
                    pose = self._lookup_current_pose()
                    distance = _xy_distance_to_waypoint(waypoint, pose)
                    if best_distance is None or distance < best_distance:
                        best_distance = distance

                    if distance <= self.success_check_distance:
                        self.node.get_logger().info(
                            "Success doublecheck passed for "
                            f"'{waypoint['name']}': final_xy_error={distance:.3f} m "
                            f"(threshold={self.success_check_distance:.3f} m)."
                        )
                        return True
                else:
                    last_error = RuntimeError(
                        f"TF {self.map_frame} <- {self.base_frame} not ready yet"
                    )
            except Exception as exc:
                last_error = exc

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))

        if best_distance is not None:
            self.node.get_logger().error(
                "Success doublecheck failed for "
                f"'{waypoint['name']}': best_final_xy_error={best_distance:.3f} m "
                f"(threshold={self.success_check_distance:.3f} m)."
            )
        else:
            self.node.get_logger().error(
                "Success doublecheck failed for "
                f"'{waypoint['name']}': could not lookup TF "
                f"{self.map_frame} <- {self.base_frame}. Last error: {last_error}"
            )

        return False


def _prompt_for_waypoint(waypoints):
    while True:
        _print_waypoints(waypoints)
        try:
            raw = input(
                "\n[waypoint_nav] choose waypoint by index or name "
                "(or id) "
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

    try:
        batch_waypoints = _resolve_waypoint_batch(config, waypoints)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        _print_waypoints(waypoints)
        raise SystemExit(1)

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
        config["abort_replan_retries"],
        config["near_goal_distance"],
        config["ignore_waypoint_yaw"],
        config["map_frame"],
        config["base_frame"],
        config["success_check_distance"],
        config["success_check_timeout"],
    )
    exit_code = 0

    try:
        if batch_waypoints is not None:
            batch_results = []
            total = len(batch_waypoints)

            for index, selected in enumerate(batch_waypoints, start=1):
                print(
                    f"[waypoint_nav] batch {index}/{total}: "
                    f"id={selected['id']} name={selected['name']}",
                    flush=True,
                )
                success = navigator.navigate_to(selected)
                batch_results.append((selected, success))
                if not success:
                    exit_code = 1
                    if not config["continue_on_failure"]:
                        break
                if index < total and config["batch_inter_goal_delay"] > 0.0:
                    print(
                        "[waypoint_nav] waiting "
                        f"{config['batch_inter_goal_delay']:.1f}s before next batch waypoint",
                        flush=True,
                    )
                    _spin_delay(node, config["batch_inter_goal_delay"])

            print("\n[waypoint_nav] batch summary:", flush=True)
            for selected, success in batch_results:
                status = "OK" if success else "FAILED"
                print(
                    f"  - {status:<6} id={selected['id']:<12} name={selected['name']}",
                    flush=True,
                )
        else:
            had_failure = False
            selected_once = waypoint is not None

            while rclpy.ok():
                selected = waypoint or _prompt_for_waypoint(waypoints)
                waypoint = None

                if selected is None:
                    break

                success = navigator.navigate_to(selected)
                had_failure = had_failure or (not success)
                if selected_once:
                    if not success:
                        exit_code = 1
                    break

            if had_failure:
                exit_code = 1
    except KeyboardInterrupt:
        navigator.cancel_active_goal()
        exit_code = 130
    finally:
        navigator.destroy()
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
