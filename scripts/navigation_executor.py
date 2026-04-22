#!/usr/bin/env python3
"""
Navigation executor for the external navigation manager.

Responsibilities:
  - connect to the manager over WebSocket
  - resolve target_id -> waypoint pose from YAML
  - send Nav2 NavigateToPose goals
  - report progress / arrived / error events
  - support abort + restart through an explicit state machine
"""

import asyncio
import contextlib
from dataclasses import dataclass
from enum import Enum
import fcntl
import json
import math
import os
import sys
import threading
import time


def _get_env_text(name, default):
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return os.path.expanduser(value)


DEFAULT_SERVER_URI = _get_env_text(
    "GO2W_SERVER_URI",
    "ws://192.168.0.131:8100/ws/navigation/executor",
)
DEFAULT_WAYPOINT_FILE = _get_env_text(
    "GO2W_WAYPOINT_FILE",
    "/home/unitree/ros_ws/src/go2w_real/config/go2w_map_waypoints.yaml",
)
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_SERVER_TIMEOUT = 10.0
DEFAULT_HEARTBEAT_INTERVAL = 30.0
DEFAULT_RECONNECT_DELAY = 3.0
DEFAULT_ABORT_WAIT_TIMEOUT = 5.0
DEFAULT_PROGRESS_INTERVAL = 1.0
DEFAULT_NAV_EVENT_TIMEOUT = 15.0
# Remote execution tends to see brief Nav2 aborts in the real robot stack
# (for example when costmaps/TF are still settling). Give the executor a
# slightly larger retry budget than the local terminal helper so it does not
# report a hard failure to the server too eagerly.
DEFAULT_ABORT_REPLAN_RETRIES = 3
DEFAULT_IGNORE_WAYPOINT_YAW = True
DEFAULT_INSTANCE_LOCK_FILE = "/tmp/go2w_navigation_executor.lock"
DEFAULT_PLANNER_ACTION_NAME = "/compute_path_to_pose"


class ExecutorState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ABORTING = "aborting"
    ERROR = "error"


@dataclass
class NavigationTask:
    request_id: str
    sub_id: int
    target_id: str
    waypoint: dict
    done_future: asyncio.Future
    token: int = 0
    abort_requested: bool = False
    last_remaining_distance: float = 0.0
    last_progress_sent_at: float = 0.0
    accepted_at: float = 0.0
    last_nav_event_at: float = 0.0
    nav_abort_retries_used: int = 0


def _print_help():
    print(
        f"Usage: ros2 run go2w_real navigation_executor.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        f"  --server-uri URI          WebSocket URI, default: {DEFAULT_SERVER_URI}\n"
        f"  --waypoint-file PATH      Waypoint YAML path, default: {DEFAULT_WAYPOINT_FILE}\n"
        f"  --action-name NAME        Nav2 action name, default: {DEFAULT_ACTION_NAME}\n"
        f"  --server-timeout SEC      Nav2 action server wait timeout, default: {DEFAULT_SERVER_TIMEOUT:.1f}\n"
        f"  --heartbeat-interval SEC  Heartbeat interval, default: {DEFAULT_HEARTBEAT_INTERVAL:.1f}\n"
        f"  --reconnect-delay SEC     Reconnect delay after disconnect, default: {DEFAULT_RECONNECT_DELAY:.1f}\n"
        f"  --nav-event-timeout SEC   Max wait for Nav2 feedback/result after goal acceptance, default: {DEFAULT_NAV_EVENT_TIMEOUT:.1f}\n"
        f"  --abort-replan-retries N  Retry after Nav2 abort if planner can still find a path, default: {DEFAULT_ABORT_REPLAN_RETRIES}\n"
        "  --use-waypoint-yaw        Respect stored waypoint yaw instead of stopping on XY only\n"
        "  --list-pois               Print the loaded POI table and exit\n"
        "  -h, --help                Show this help message\n"
        "\n"
        "Environment overrides:\n"
        "  GO2W_SERVER_URI\n"
        "  GO2W_WAYPOINT_FILE\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "server_uri": DEFAULT_SERVER_URI,
        "waypoint_file": DEFAULT_WAYPOINT_FILE,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout": DEFAULT_SERVER_TIMEOUT,
        "heartbeat_interval": DEFAULT_HEARTBEAT_INTERVAL,
        "reconnect_delay": DEFAULT_RECONNECT_DELAY,
        "nav_event_timeout": DEFAULT_NAV_EVENT_TIMEOUT,
        "abort_replan_retries": DEFAULT_ABORT_REPLAN_RETRIES,
        "ignore_waypoint_yaw": DEFAULT_IGNORE_WAYPOINT_YAW,
        "list_pois": False,
    }
    ros_args = []

    i = 0
    while i < len(raw_args):
        arg = raw_args[i]

        if arg in ("-h", "--help"):
            _print_help()
            raise SystemExit(0)
        if arg == "--server-uri" and i + 1 < len(raw_args):
            config["server_uri"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--server-uri="):
            config["server_uri"] = arg.split("=", 1)[1]
            i += 1
            continue
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
        if arg == "--heartbeat-interval" and i + 1 < len(raw_args):
            config["heartbeat_interval"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--heartbeat-interval="):
            config["heartbeat_interval"] = float(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--reconnect-delay" and i + 1 < len(raw_args):
            config["reconnect_delay"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--reconnect-delay="):
            config["reconnect_delay"] = float(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--nav-event-timeout" and i + 1 < len(raw_args):
            config["nav_event_timeout"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--nav-event-timeout="):
            config["nav_event_timeout"] = float(arg.split("=", 1)[1])
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
        if arg == "--use-waypoint-yaw":
            config["ignore_waypoint_yaw"] = False
            i += 1
            continue
        if arg == "--list-pois":
            config["list_pois"] = True
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


def _apply_waypoint_to_pose_stamped(
    pose_stamped,
    waypoint,
    stamp,
    ignore_waypoint_yaw=DEFAULT_IGNORE_WAYPOINT_YAW,
):
    pose_stamped.header.frame_id = waypoint["frame_id"]
    pose_stamped.header.stamp = stamp
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


def _goal_status_label(status):
    status = int(status)
    labels = {
        0: "unknown",
        1: "accepted",
        2: "executing",
        3: "canceling",
        4: "succeeded",
        5: "canceled",
        6: "aborted",
    }
    return labels.get(status, f"status_{status}")


def _load_waypoints(path):
    import yaml

    waypoint_path = os.path.expanduser(path)
    with open(waypoint_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    raw_waypoints = data.get("waypoints", [])
    if not isinstance(raw_waypoints, list):
        raise ValueError(
            f"Invalid waypoint file: 'waypoints' is not a list in {waypoint_path}"
        )

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

        name = str(raw_wp.get("name", f"wp_{idx:02d}"))
        waypoint_id = str(raw_wp.get("id", name))

        waypoints.append(
            {
                "index": idx,
                "id": waypoint_id,
                "name": name,
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


def _resolve_waypoint_by_target_id(target_id, waypoints):
    text = str(target_id).strip()
    if not text:
        return None

    lowered = text.lower()
    for wp in waypoints:
        if wp["id"].lower() == lowered:
            return wp
    for wp in waypoints:
        if wp["name"].lower() == lowered:
            return wp
    if text.isdigit():
        index = int(text)
        for wp in waypoints:
            if wp["index"] == index:
                return wp
    return None


def _print_waypoints(waypoints):
    print("\nLoaded POIs:", flush=True)
    if not waypoints:
        print("  (none)", flush=True)
        return

    for wp in waypoints:
        print(
            f"  {wp['index']:>2}. id={wp['id']:<12} "
            f"name={wp['name']:<16} "
            f"x={wp['position']['x']:.3f} "
            f"y={wp['position']['y']:.3f} "
            f"yaw={math.degrees(wp['yaw']):.1f} deg",
            flush=True,
        )


def _acquire_instance_lock(lock_file_path):
    lock_path = os.path.expanduser(lock_file_path)
    lock_handle = open(lock_path, "a+", encoding="utf-8")

    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or "unknown"
        lock_handle.close()
        raise RuntimeError(
            "another navigation_executor instance is already running "
            f"(pid={owner_pid}, lock={lock_path})"
        ) from exc

    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(f"{os.getpid()}\n")
    lock_handle.flush()
    return lock_handle


class Nav2ActionBridge:
    def __init__(
        self,
        loop,
        event_queue,
        action_name,
        server_timeout,
        ignore_waypoint_yaw,
        ros_args,
    ):
        import rclpy
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node

        self._loop = loop
        self._event_queue = event_queue
        self._server_timeout = server_timeout
        self._ignore_waypoint_yaw = bool(ignore_waypoint_yaw)
        self._lock = threading.RLock()
        self._active_task = None
        self._active_goal_handle = None
        self._token_counter = 0

        rclpy.init(args=ros_args)
        self._rclpy = rclpy
        self._compute_path_to_pose = ComputePathToPose
        self._navigate_to_pose = NavigateToPose
        self.node = Node("navigation_executor")
        self.client = ActionClient(self.node, NavigateToPose, action_name)
        self.planner_client = ActionClient(
            self.node,
            ComputePathToPose,
            DEFAULT_PLANNER_ACTION_NAME,
        )
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin,
            name="navigation_executor_rclpy",
            daemon=True,
        )
        self._spin_thread.start()

    def shutdown(self):
        with contextlib.suppress(Exception):
            self.cancel_current()
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except TypeError:
            with contextlib.suppress(Exception):
                self.executor.shutdown()
        with contextlib.suppress(Exception):
            self.client.destroy()
        with contextlib.suppress(Exception):
            self.planner_client.destroy()
        with contextlib.suppress(Exception):
            self.executor.remove_node(self.node)
        with contextlib.suppress(Exception):
            self.node.destroy_node()
        with contextlib.suppress(Exception):
            if self._rclpy.ok():
                self._rclpy.shutdown()

    def start_navigation(self, task):
        with self._lock:
            self._token_counter += 1
            task.token = self._token_counter
            self._active_task = task
            self._active_goal_handle = None

        self.node.get_logger().info(
            f"Preparing Nav2 goal for target_id={task.target_id} "
            f"(request_id={task.request_id}, sub_id={task.sub_id})"
        )

        if task.abort_requested:
            self._clear_if_active(task.token)
            self._publish_event(
                {
                    "kind": "canceled",
                    "token": task.token,
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                    "remaining_distance": task.last_remaining_distance,
                }
            )
            return False

        if not self.client.wait_for_server(timeout_sec=self._server_timeout):
            self._clear_if_active(task.token)
            if task.abort_requested:
                self._publish_event(
                    {
                        "kind": "canceled",
                        "token": task.token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "remaining_distance": task.last_remaining_distance,
                    }
                )
            else:
                self._publish_event(
                    {
                        "kind": "error",
                        "token": task.token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "error_message": (
                            "Nav2 action server is not available within "
                            f"{self._server_timeout:.1f}s"
                        ),
                    }
                )
            return False

        with self._lock:
            if self._active_task is None or self._active_task.token != task.token:
                return False
            if task.abort_requested:
                self._active_task = None
                self._active_goal_handle = None
                self._publish_event(
                    {
                        "kind": "canceled",
                        "token": task.token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "remaining_distance": task.last_remaining_distance,
                    }
                )
                return False

        goal_msg = self._navigate_to_pose.Goal()
        _apply_waypoint_to_pose_stamped(
            goal_msg.pose,
            task.waypoint,
            self.node.get_clock().now().to_msg(),
            self._ignore_waypoint_yaw,
        )

        future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg, token=task.token: self._on_feedback(
                token, task, msg
            ),
        )
        future.add_done_callback(
            lambda fut, token=task.token, task=task: self._on_goal_response(
                token, task, fut
            )
        )
        return True

    def cancel_current(self):
        with self._lock:
            task = self._active_task
            goal_handle = self._active_goal_handle
            if task is None:
                return False
            task.abort_requested = True

        self._publish_event(
            {
                "kind": "aborting",
                "token": task.token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
            }
        )

        if goal_handle is not None:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda fut, token=task.token, task=task: self._on_cancel_response(
                    token, task, fut
                )
            )

        return True

    def _on_goal_response(self, token, task, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._clear_if_active(token)
            if task.abort_requested:
                self._publish_event(
                    {
                        "kind": "canceled",
                        "token": token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "remaining_distance": task.last_remaining_distance,
                    }
                )
            else:
                self._publish_event(
                    {
                        "kind": "error",
                        "token": token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "error_message": f"Nav2 goal send failed: {exc}",
                    }
                )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._clear_if_active(token)
            if task.abort_requested:
                self._publish_event(
                    {
                        "kind": "canceled",
                        "token": token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "remaining_distance": task.last_remaining_distance,
                    }
                )
            else:
                self._publish_event(
                    {
                        "kind": "error",
                        "token": token,
                        "request_id": task.request_id,
                        "sub_id": task.sub_id,
                        "error_message": "Nav2 rejected the goal",
                    }
                )
            return

        with self._lock:
            if self._active_task is None or self._active_task.token != token:
                goal_handle.cancel_goal_async()
                return
            self._active_goal_handle = goal_handle

        self._publish_event(
            {
                "kind": "accepted",
                "token": token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
            }
        )

        if task.abort_requested:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda fut, token=task.token, task=task: self._on_cancel_response(
                    token, task, fut
                )
            )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, token=task.token, task=task: self._on_result(token, task, fut)
        )

    def _on_feedback(self, token, task, feedback_msg):
        with self._lock:
            if self._active_task is None or self._active_task.token != token:
                return

        feedback = feedback_msg.feedback
        if not hasattr(feedback, "distance_remaining"):
            return

        remaining_distance = float(feedback.distance_remaining)
        task.last_remaining_distance = remaining_distance
        self._publish_event(
            {
                "kind": "progress",
                "token": token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "remaining_distance": remaining_distance,
                "status": "running",
            }
        )

    def _on_cancel_response(self, token, task, future):
        try:
            future.result()
        except Exception as exc:
            self._clear_if_active(token)
            self._publish_event(
                {
                    "kind": "error",
                    "token": token,
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                    "error_message": f"Nav2 cancel failed: {exc}",
                }
            )

    def _on_result(self, token, task, future):
        from action_msgs.msg import GoalStatus

        try:
            result = future.result()
        except Exception as exc:
            self._clear_if_active(token)
            self._publish_event(
                {
                    "kind": "error",
                    "token": token,
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                    "error_message": f"Nav2 result wait failed: {exc}",
                }
            )
            return

        self._clear_if_active(token)

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_event(
                {
                    "kind": "arrived",
                    "token": token,
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                }
            )
            return

        if result.status == GoalStatus.STATUS_CANCELED or task.abort_requested:
            self._publish_event(
                {
                    "kind": "canceled",
                    "token": token,
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                    "remaining_distance": task.last_remaining_distance,
                }
            )
            return

        self._publish_event(
            {
                "kind": "error",
                "token": token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "nav_status": int(result.status),
                "error_message": (
                    f"Nav2 finished with status {result.status} "
                    f"({_goal_status_label(result.status)})"
                ),
            }
        )

    def planner_has_path(self, waypoint):
        from action_msgs.msg import GoalStatus

        if not self.planner_client.wait_for_server(timeout_sec=self._server_timeout):
            raise RuntimeError(
                "ComputePathToPose action server is not available within "
                f"{self._server_timeout:.1f}s"
            )

        goal_msg = self._compute_path_to_pose.Goal()
        _apply_waypoint_to_pose_stamped(
            goal_msg.pose,
            waypoint,
            self.node.get_clock().now().to_msg(),
            self._ignore_waypoint_yaw,
        )
        goal_msg.planner_id = "GridBased"

        goal_handle = self._wait_for_future(
            self.planner_client.send_goal_async(goal_msg),
            self._server_timeout,
            "ComputePathToPose goal response",
        )
        if goal_handle is None or not goal_handle.accepted:
            return False

        result = self._wait_for_future(
            goal_handle.get_result_async(),
            self._server_timeout,
            "ComputePathToPose result",
        )
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return False

        path = getattr(result.result, "path", None)
        poses = getattr(path, "poses", None)
        return bool(poses)

    def _wait_for_future(self, future, timeout_sec, label):
        done = threading.Event()
        outcome = {}

        def _finish(completed_future):
            try:
                outcome["result"] = completed_future.result()
            except Exception as exc:
                outcome["error"] = exc
            finally:
                done.set()

        future.add_done_callback(_finish)
        if not done.wait(timeout_sec):
            raise TimeoutError(
                f"Timed out waiting for {label} within {timeout_sec:.1f}s"
            )
        if "error" in outcome:
            raise outcome["error"]
        return outcome.get("result")

    def _clear_if_active(self, token):
        with self._lock:
            if self._active_task is not None and self._active_task.token == token:
                self._active_task = None
                self._active_goal_handle = None

    def _publish_event(self, event):
        try:
            self._loop.call_soon_threadsafe(self._event_queue.put_nowait, event)
        except RuntimeError:
            pass


class NavigationExecutor:
    def __init__(self, config, ros_args):
        self.config = config
        self.ros_args = ros_args
        self.websocket = None
        self.send_lock = None
        self.state = ExecutorState.DISCONNECTED
        self.current_task = None
        self._bridge_event_queue = None
        self._loop = None
        self._bridge = None
        self._message_tasks = set()
        self._command_version = 0

    async def run(self):
        import websockets

        self._loop = asyncio.get_running_loop()
        self.send_lock = asyncio.Lock()
        self._bridge_event_queue = asyncio.Queue()
        self._bridge = Nav2ActionBridge(
            self._loop,
            self._bridge_event_queue,
            self.config["action_name"],
            self.config["server_timeout"],
            self.config.get("ignore_waypoint_yaw", DEFAULT_IGNORE_WAYPOINT_YAW),
            self.ros_args,
        )

        bridge_task = asyncio.create_task(self._process_bridge_events())
        try:
            while True:
                self._set_state(ExecutorState.CONNECTING)
                try:
                    async with websockets.connect(
                        self.config["server_uri"],
                        ping_interval=None,
                    ) as websocket:
                        self.websocket = websocket
                        if self.current_task is None:
                            self._set_state(ExecutorState.IDLE)
                        print(
                            f"[navigation_executor] connected to {self.config['server_uri']}",
                            flush=True,
                        )

                        receiver_task = asyncio.create_task(
                            self._receive_messages(websocket)
                        )
                        heartbeat_task = asyncio.create_task(
                            self._send_heartbeat(websocket)
                        )
                        done, pending = await asyncio.wait(
                            [receiver_task, heartbeat_task],
                            return_when=asyncio.FIRST_COMPLETED,
                        )
                        for task in pending:
                            task.cancel()
                            with contextlib.suppress(asyncio.CancelledError):
                                await task
                        for task in done:
                            exc = task.exception()
                            if exc is not None:
                                raise exc
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    print(
                        f"[navigation_executor] websocket disconnected: {exc}",
                        flush=True,
                    )
                finally:
                    self.websocket = None
                    self._command_version += 1
                    await self._cancel_message_tasks()
                    await self._abort_current(
                        expected_request_id=None,
                        report_pause=False,
                        reason="connection lost",
                    )
                    self._set_state(ExecutorState.DISCONNECTED)

                await asyncio.sleep(self.config["reconnect_delay"])
        finally:
            await self._cancel_message_tasks()
            bridge_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await bridge_task
            self._bridge.shutdown()

    async def _receive_messages(self, websocket):
        async for raw_message in websocket:
            try:
                data = json.loads(raw_message)
            except json.JSONDecodeError:
                print(
                    f"[navigation_executor] ignored invalid JSON: {raw_message}",
                    flush=True,
                )
                continue

            if data.get("type") == "ping":
                await self._send_json({"type": "pong"})
                continue
            if data.get("type") == "pong":
                continue

            action = data.get("action")
            if action == "navigate_to":
                self._create_background_task(
                    self._handle_navigate_to(data),
                    label="navigate_to",
                )
            elif action == "abort_navigation":
                self._create_background_task(
                    self._handle_abort_navigation(data),
                    label="abort_navigation",
                )
            else:
                print(
                    f"[navigation_executor] ignored unsupported message: {data}",
                    flush=True,
                )

    async def _send_heartbeat(self, websocket):
        while True:
            await asyncio.sleep(self.config["heartbeat_interval"])
            if websocket.closed:
                return
            await self._send_json({"type": "ping"})

    async def _process_bridge_events(self):
        while True:
            event = await self._bridge_event_queue.get()
            try:
                await self._handle_bridge_event(event)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                print(
                    f"[navigation_executor] bridge event handling failed: {exc}",
                    flush=True,
                )

    async def _handle_navigate_to(self, data):
        command_version = self._next_command_version()
        request_id = str(data.get("request_id", "")).strip()
        target_id = str(data.get("target_id", "")).strip()

        try:
            sub_id = int(data.get("sub_id"))
        except (TypeError, ValueError):
            sub_id = None

        if not request_id or sub_id is None or not target_id:
            await self._send_error_event(
                request_id or "unknown",
                sub_id if sub_id is not None else -1,
                "invalid navigate_to payload",
            )
            return

        print(
            "[navigation_executor] received navigate_to: "
            f"request_id={request_id}, sub_id={sub_id}, target_id={target_id}",
            flush=True,
        )

        try:
            waypoints = _load_waypoints(self.config["waypoint_file"])
        except Exception as exc:
            await self._send_error_event(
                request_id,
                sub_id,
                f"failed to load waypoint file: {exc}",
            )
            self._set_state(ExecutorState.ERROR)
            return

        waypoint = _resolve_waypoint_by_target_id(target_id, waypoints)
        if waypoint is None:
            await self._send_error_event(
                request_id,
                sub_id,
                f"unknown target_id '{target_id}'",
            )
            self._set_state(ExecutorState.ERROR)
            return

        if self.current_task is not None:
            await self._abort_current(
                expected_request_id=None,
                report_pause=True,
                reason=f"preempted by new navigate_to({request_id})",
            )

        if not self._is_latest_command(command_version):
            return

        done_future = self._loop.create_future()
        task = NavigationTask(
            request_id=request_id,
            sub_id=sub_id,
            target_id=target_id,
            waypoint=waypoint,
            done_future=done_future,
        )
        self.current_task = task
        self._set_state(ExecutorState.STARTING)
        self._create_background_task(
            self._start_navigation(task, command_version),
            label=f"start_navigation:{request_id}",
        )
        self._create_background_task(
            self._watch_navigation_feedback(task, command_version),
            label=f"watch_navigation:{request_id}",
        )

    async def _start_navigation(self, task, command_version):
        if not self._is_latest_command(command_version):
            return
        if self.current_task is not task:
            return

        try:
            await self._run_blocking(self._bridge.start_navigation, task)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if self.current_task is task:
                await self._send_error_event(
                    task.request_id,
                    task.sub_id,
                    f"failed to start navigation: {exc}",
                )
                self._complete_task(task, set_idle=False)
                self._set_state(ExecutorState.ERROR)

    async def _handle_abort_navigation(self, data):
        self._next_command_version()
        request_id = str(data.get("request_id", "")).strip()
        print(
            "[navigation_executor] received abort_navigation: "
            f"request_id={request_id or 'unknown'}",
            flush=True,
        )
        await self._abort_current(
            expected_request_id=request_id or None,
            report_pause=True,
            reason="abort_navigation command",
        )

    async def _abort_current(self, expected_request_id, report_pause, reason):
        task = self.current_task
        if task is None:
            return False

        if expected_request_id and task.request_id != expected_request_id:
            print(
                "[navigation_executor] ignore abort for request_id="
                f"{expected_request_id}, current is {task.request_id}",
                flush=True,
            )
            return False

        task.abort_requested = True
        self._set_state(ExecutorState.ABORTING)
        print(f"[navigation_executor] aborting current goal: {reason}", flush=True)

        try:
            await self._run_blocking(self._bridge.cancel_current)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(
                f"[navigation_executor] cancel_current failed: {exc}",
                flush=True,
            )

        try:
            await asyncio.wait_for(
                asyncio.shield(task.done_future),
                timeout=DEFAULT_ABORT_WAIT_TIMEOUT,
            )
        except asyncio.TimeoutError:
            print(
                "[navigation_executor] cancel wait timed out; continuing anyway",
                flush=True,
            )
            if report_pause:
                await self._send_progress_event(
                    task.request_id,
                    task.sub_id,
                    task.last_remaining_distance,
                    "paused",
                )
            self._complete_task(task, set_idle=True)

        return True

    async def _handle_bridge_event(self, event):
        task = self.current_task
        if task is None or task.token != event.get("token"):
            return

        kind = event["kind"]
        if kind == "accepted":
            now = time.monotonic()
            task.accepted_at = now
            task.last_nav_event_at = now
            if not task.abort_requested:
                self._set_state(ExecutorState.NAVIGATING)
            return

        if kind == "aborting":
            self._set_state(ExecutorState.ABORTING)
            return

        if kind == "progress":
            now = time.monotonic()
            remaining_distance = float(event.get("remaining_distance", 0.0))
            task.last_remaining_distance = remaining_distance
            task.last_nav_event_at = now

            if task.abort_requested:
                return

            if now - task.last_progress_sent_at < DEFAULT_PROGRESS_INTERVAL:
                return

            task.last_progress_sent_at = now
            await self._send_progress_event(
                task.request_id,
                task.sub_id,
                remaining_distance,
                event.get("status", "running"),
            )
            return

        if kind == "arrived":
            await self._send_json(
                {
                    "event_type": "on_arrived",
                    "request_id": task.request_id,
                    "sub_id": task.sub_id,
                }
            )
            self._complete_task(task, set_idle=True)
            return

        if kind == "canceled":
            remaining_distance = float(
                event.get("remaining_distance", task.last_remaining_distance)
            )
            task.last_remaining_distance = remaining_distance
            await self._send_progress_event(
                task.request_id,
                task.sub_id,
                remaining_distance,
                "paused",
            )
            self._complete_task(task, set_idle=True)
            return

        if kind == "error":
            if await self._maybe_retry_after_nav_abort(task, event):
                return
            await self._send_error_event(
                task.request_id,
                task.sub_id,
                event.get("error_message", "navigation error"),
            )
            self._complete_task(task, set_idle=False)
            self._set_state(ExecutorState.ERROR)
            return

    async def _maybe_retry_after_nav_abort(self, task, event):
        nav_status = int(event.get("nav_status", 0) or 0)
        if _goal_status_label(nav_status) != "aborted":
            return False
        if task.abort_requested:
            return False

        max_retries = max(
            0,
            int(
                self.config.get(
                    "abort_replan_retries",
                    DEFAULT_ABORT_REPLAN_RETRIES,
                )
            ),
        )
        if task.nav_abort_retries_used >= max_retries:
            return False

        next_attempt = task.nav_abort_retries_used + 1
        print(
            "[navigation_executor] Nav2 aborted; probing planner for a retryable "
            f"path (attempt {next_attempt}/{max_retries})",
            flush=True,
        )

        task.accepted_at = 0.0
        task.last_nav_event_at = time.monotonic()
        self._set_state(ExecutorState.STARTING)

        try:
            has_path = await self._run_blocking(
                self._bridge.planner_has_path,
                task.waypoint,
            )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self._send_error_event(
                task.request_id,
                task.sub_id,
                f"Nav2 aborted and replanning probe failed: {exc}",
            )
            self._complete_task(task, set_idle=False)
            self._set_state(ExecutorState.ERROR)
            return True

        if self.current_task is not task or task.abort_requested:
            return True

        if not has_path:
            await self._send_error_event(
                task.request_id,
                task.sub_id,
                "Nav2 aborted and planner could not find a path to the target",
            )
            self._complete_task(task, set_idle=False)
            self._set_state(ExecutorState.ERROR)
            return True

        task.nav_abort_retries_used = next_attempt
        print(
            "[navigation_executor] planner still has a path; retrying current goal "
            f"(attempt {next_attempt}/{max_retries})",
            flush=True,
        )

        try:
            await self._run_blocking(self._bridge.start_navigation, task)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self._send_error_event(
                task.request_id,
                task.sub_id,
                f"failed to restart navigation after Nav2 abort: {exc}",
            )
            self._complete_task(task, set_idle=False)
            self._set_state(ExecutorState.ERROR)
        return True

    async def _watch_navigation_feedback(self, task, command_version):
        timeout = float(self.config["nav_event_timeout"])
        if timeout <= 0.0:
            return

        while True:
            await asyncio.sleep(0.5)

            if not self._is_latest_command(command_version):
                return
            if self.current_task is not task:
                return
            if task.done_future.done() or task.abort_requested:
                return
            if task.accepted_at <= 0.0:
                continue

            idle_for = time.monotonic() - task.last_nav_event_at
            if idle_for < timeout:
                continue

            error_message = (
                "Nav2 accepted the goal but produced no feedback/result for "
                f"{timeout:.1f}s; planner may be stuck or the target may be "
                "unreachable"
            )
            print(
                "[navigation_executor] navigation feedback watchdog fired: "
                f"request_id={task.request_id}, target_id={task.target_id}, "
                f"idle_for={idle_for:.1f}s",
                flush=True,
            )
            await self._send_error_event(
                task.request_id,
                task.sub_id,
                error_message,
            )

            task.abort_requested = True
            try:
                await self._run_blocking(self._bridge.cancel_current)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                print(
                    f"[navigation_executor] watchdog cancel_current failed: {exc}",
                    flush=True,
                )

            if self.current_task is task:
                self._complete_task(task, set_idle=False)
                self._set_state(ExecutorState.ERROR)
            return

    async def _send_progress_event(self, request_id, sub_id, remaining_distance, status):
        await self._send_json(
            {
                "event_type": "on_progress",
                "request_id": request_id,
                "sub_id": sub_id,
                "remaining_distance": round(float(remaining_distance), 3),
                "status": status,
            }
        )

    async def _send_error_event(self, request_id, sub_id, error_message):
        await self._send_json(
            {
                "event_type": "on_error",
                "request_id": request_id,
                "sub_id": sub_id,
                "error_message": str(error_message),
            }
        )

    async def _send_json(self, payload):
        if self.websocket is None:
            return False

        async with self.send_lock:
            if self.websocket is None:
                return False
            await self.websocket.send(json.dumps(payload, ensure_ascii=False))
        return True

    def _set_state(self, new_state):
        if self.state == new_state:
            return
        print(
            f"[navigation_executor] state: {self.state.value} -> {new_state.value}",
            flush=True,
        )
        self.state = new_state

    def _complete_task(self, task, set_idle):
        if self.current_task is task:
            self.current_task = None
        if not task.done_future.done():
            task.done_future.set_result(True)
        if set_idle:
            self._set_state(ExecutorState.IDLE)

    def _next_command_version(self):
        self._command_version += 1
        return self._command_version

    def _is_latest_command(self, version):
        return version == self._command_version

    def _create_background_task(self, coro, label):
        task = asyncio.create_task(coro)
        self._message_tasks.add(task)
        task.add_done_callback(
            lambda done_task, task_label=label: self._background_task_done(
                done_task, task_label
            )
        )
        return task

    def _background_task_done(self, task, label):
        self._message_tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is None:
            return
        print(f"[navigation_executor] {label} failed: {exc}", flush=True)

    async def _cancel_message_tasks(self):
        if not self._message_tasks:
            return

        tasks = list(self._message_tasks)
        for task in tasks:
            task.cancel()

        await asyncio.gather(*tasks, return_exceptions=True)

        self._message_tasks.clear()

    async def _run_blocking(self, func, *args):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, lambda: func(*args))


def main(args=None):
    if args is None:
        raw_args = sys.argv[1:]
    else:
        raw_args = list(args)

    config, ros_args = _parse_cli_args(raw_args)
    waypoints = _load_waypoints(config["waypoint_file"])

    if config["list_pois"]:
        _print_waypoints(waypoints)
        return

    instance_lock = _acquire_instance_lock(DEFAULT_INSTANCE_LOCK_FILE)
    executor = NavigationExecutor(config, ros_args)
    try:
        asyncio.run(executor.run())
    except KeyboardInterrupt:
        pass
    finally:
        instance_lock.close()


if __name__ == "__main__":
    main()
