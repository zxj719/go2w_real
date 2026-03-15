#!/usr/bin/env python3
"""Unitree SLAM service client using pure ROS2 pub/sub.

Replaces the SDK2-based SlamClient with ROS2 topic communication
to avoid CycloneDDS conflicts on the robot Jetson.

Publishes to:  /api/slam_operate/request  (unitree_api/msg/Request)
Subscribes to: /api/slam_operate/response (unitree_api/msg/Response)

IMPORTANT: The parent node must be spun with a MultiThreadedExecutor
so the response subscription callback can fire while _call() blocks.
"""

import json
import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from unitree_api.msg import Request, Response

# ---------------------------------------------------------------------------
# Service constants (from keyDemo.cpp)
# ---------------------------------------------------------------------------
SLAM_API_ID_START_MAPPING = 1801
SLAM_API_ID_END_MAPPING = 1802
SLAM_API_ID_START_RELOCATION = 1804
SLAM_API_ID_NAVIGATE = 1102
SLAM_API_ID_PAUSE_NAV = 1201
SLAM_API_ID_RESUME_NAV = 1202
SLAM_API_ID_STOP_SLAM = 1901


class SlamClient:
    """ROS2 pub/sub client for the Unitree SLAM service."""

    def __init__(self, node: Node, timeout: float = 10.0):
        self._node = node
        self._timeout = timeout
        self._request_id = 0
        self._lock = threading.Lock()
        self._pending = {}  # id -> (threading.Event, result_tuple)

        # Use a separate callback group so response_cb can fire
        # while the main timer callback blocks on event.wait()
        self._cb_group = MutuallyExclusiveCallbackGroup()

        self._pub = node.create_publisher(
            Request, '/api/slam_operate/request', 10)
        self._sub = node.create_subscription(
            Response, '/api/slam_operate/response',
            self._response_cb, 10,
            callback_group=self._cb_group)

    def _response_cb(self, msg: Response):
        req_id = msg.header.identity.id
        with self._lock:
            if req_id in self._pending:
                event, _ = self._pending[req_id]
                self._pending[req_id] = (event, (
                    msg.header.status.code, msg.data))
                event.set()

    def _call(self, api_id: int, parameter: str) -> tuple:
        with self._lock:
            self._request_id += 1
            req_id = self._request_id
            event = threading.Event()
            self._pending[req_id] = (event, None)

        msg = Request()
        msg.header.identity.id = req_id
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        self._pub.publish(msg)

        if event.wait(timeout=self._timeout):
            with self._lock:
                _, result = self._pending.pop(req_id)
            return result
        else:
            with self._lock:
                self._pending.pop(req_id, None)
            self._node.get_logger().error(
                f'SLAM API {api_id} timed out after {self._timeout}s')
            return (-1, '')

    # -- Mapping --------------------------------------------------------------

    def StartMapping(self, slam_type: str = "indoor"):
        p = {"data": {"slam_type": slam_type}}
        return self._call(SLAM_API_ID_START_MAPPING, json.dumps(p))

    def EndMapping(self, save_path: str = "/home/unitree/auto_explore.pcd"):
        p = {"data": {"address": save_path}}
        return self._call(SLAM_API_ID_END_MAPPING, json.dumps(p))

    # -- Relocation -----------------------------------------------------------

    def StartRelocation(
        self,
        x: float = 0.0, y: float = 0.0, z: float = 0.0,
        q_x: float = 0.0, q_y: float = 0.0, q_z: float = 0.0, q_w: float = 1.0,
        address: str = "/home/unitree/auto_explore.pcd",
    ):
        p = {
            "data": {
                "x": x, "y": y, "z": z,
                "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w,
                "address": address,
            }
        }
        return self._call(SLAM_API_ID_START_RELOCATION, json.dumps(p))

    # -- Navigation -----------------------------------------------------------

    def NavigateTo(
        self,
        x: float, y: float, z: float = 0.0,
        q_x: float = 0.0, q_y: float = 0.0, q_z: float = 0.0, q_w: float = 1.0,
        mode: int = 0, speed: float = 0.8,
    ):
        p = {
            "data": {
                "targetPose": {
                    "x": x, "y": y, "z": z,
                    "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w,
                },
                "mode": mode,
                "speed": speed,
            }
        }
        return self._call(SLAM_API_ID_NAVIGATE, json.dumps(p))

    def PauseNavigation(self):
        p = {"data": {}}
        return self._call(SLAM_API_ID_PAUSE_NAV, json.dumps(p))

    def ResumeNavigation(self):
        p = {"data": {}}
        return self._call(SLAM_API_ID_RESUME_NAV, json.dumps(p))

    # -- Lifecycle ------------------------------------------------------------

    def StopSlam(self):
        p = {"data": {}}
        return self._call(SLAM_API_ID_STOP_SLAM, json.dumps(p))
