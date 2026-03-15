#!/usr/bin/env python3
"""
Standalone autonomous frontier explorer for GO2W using Unitree SDK2 only.

Runs entirely on the robot Jetson — NO ROS2/rclpy dependency.
Uses SDK2 DDS for:
  - SlamClient RPC (start mapping, navigate, stop)
  - ChannelSubscriber on rt/slam_info for robot pose
  - ChannelSubscriber on rt/slam_key_info for arrival feedback
  - ChannelSubscriber on rt/utlidar/cloud for point cloud → occupancy grid

Usage:
  python3 auto_explore_standalone.py [network_interface]
  python3 auto_explore_standalone.py eth0
"""

import json
import math
import struct
import sys
import threading
import time
from collections import deque

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.rpc.client import Client


# ===================================================================
# SlamClient — Unitree SLAM RPC service
# ===================================================================
SLAM_SERVICE_NAME = "slam_operate"
SLAM_API_VERSION = "1.0.0.1"
SLAM_API_ID_START_MAPPING = 1801
SLAM_API_ID_END_MAPPING = 1802
SLAM_API_ID_START_RELOCATION = 1804
SLAM_API_ID_NAVIGATE = 1102
SLAM_API_ID_PAUSE_NAV = 1201
SLAM_API_ID_RESUME_NAV = 1202
SLAM_API_ID_STOP_SLAM = 1901


class SlamClient(Client):
    def __init__(self):
        super().__init__(SLAM_SERVICE_NAME, False)

    def Init(self):
        self._SetApiVerson(SLAM_API_VERSION)
        self._RegistApi(SLAM_API_ID_START_MAPPING, 0)
        self._RegistApi(SLAM_API_ID_END_MAPPING, 0)
        self._RegistApi(SLAM_API_ID_START_RELOCATION, 0)
        self._RegistApi(SLAM_API_ID_NAVIGATE, 0)
        self._RegistApi(SLAM_API_ID_PAUSE_NAV, 0)
        self._RegistApi(SLAM_API_ID_RESUME_NAV, 0)
        self._RegistApi(SLAM_API_ID_STOP_SLAM, 0)

    def StartMapping(self, slam_type="indoor"):
        p = {"data": {"slam_type": slam_type}}
        return self._Call(SLAM_API_ID_START_MAPPING, json.dumps(p))

    def EndMapping(self, save_path="/home/unitree/auto_explore.pcd"):
        p = {"data": {"address": save_path}}
        return self._Call(SLAM_API_ID_END_MAPPING, json.dumps(p))

    def NavigateTo(self, x, y, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0,
                   mode=0, speed=0.8):
        p = {"data": {"targetPose": {"x": x, "y": y, "z": z,
                                     "q_x": q_x, "q_y": q_y,
                                     "q_z": q_z, "q_w": q_w},
                      "mode": mode, "speed": speed}}
        return self._Call(SLAM_API_ID_NAVIGATE, json.dumps(p))

    def PauseNavigation(self):
        return self._Call(SLAM_API_ID_PAUSE_NAV, json.dumps({"data": {}}))

    def ResumeNavigation(self):
        return self._Call(SLAM_API_ID_RESUME_NAV, json.dumps({"data": {}}))

    def StopSlam(self):
        return self._Call(SLAM_API_ID_STOP_SLAM, json.dumps({"data": {}}))


# ===================================================================
# Frontier data container
# ===================================================================
class Frontier:
    __slots__ = ('size', 'min_distance', 'cost', 'centroid', 'points')

    def __init__(self):
        self.size = 0
        self.min_distance = float('inf')
        self.cost = 0.0
        self.centroid = (0.0, 0.0)
        self.points = []


# ===================================================================
# OccupancyGrid — built from point cloud + known pose
# ===================================================================
class OccupancyGrid:
    def __init__(self, resolution=0.05, initial_size_m=20.0, max_range=8.0):
        self.resolution = resolution
        self.max_range = max_range

        n = int(initial_size_m / resolution)
        self.grid = np.zeros((n, n), dtype=np.float32)  # log-odds
        self.origin_x = -initial_size_m / 2.0
        self.origin_y = -initial_size_m / 2.0

        self.hit_log = 0.9
        self.miss_log = -0.4
        self.clamp_min = -5.0
        self.clamp_max = 5.0

    @property
    def height(self):
        return self.grid.shape[0]

    @property
    def width(self):
        return self.grid.shape[1]

    def world_to_cell(self, wx, wy):
        return (int((wx - self.origin_x) / self.resolution),
                int((wy - self.origin_y) / self.resolution))

    def ensure_contains(self, wx, wy, margin=0.0):
        need_x_min = wx - margin
        need_x_max = wx + margin
        need_y_min = wy - margin
        need_y_max = wy + margin
        cur_x_max = self.origin_x + self.width * self.resolution
        cur_y_max = self.origin_y + self.height * self.resolution

        if (need_x_min >= self.origin_x and need_x_max <= cur_x_max
                and need_y_min >= self.origin_y and need_y_max <= cur_y_max):
            return

        pad = margin * 0.2
        new_x_min = min(self.origin_x, need_x_min - pad)
        new_y_min = min(self.origin_y, need_y_min - pad)
        new_x_max = max(cur_x_max, need_x_max + pad)
        new_y_max = max(cur_y_max, need_y_max + pad)
        new_w = int(math.ceil((new_x_max - new_x_min) / self.resolution))
        new_h = int(math.ceil((new_y_max - new_y_min) / self.resolution))

        new_grid = np.zeros((new_h, new_w), dtype=np.float32)
        off_c = int(round((self.origin_x - new_x_min) / self.resolution))
        off_r = int(round((self.origin_y - new_y_min) / self.resolution))
        old_h, old_w = self.grid.shape
        new_grid[off_r:off_r + old_h, off_c:off_c + old_w] = self.grid

        self.grid = new_grid
        self.origin_x = new_x_min
        self.origin_y = new_y_min
        print(f"[Grid] Expanded to {new_w}x{new_h}")

    def update_from_points(self, robot_x, robot_y, robot_yaw, points_xyz):
        """Integrate 3D points into 2D grid (only floor-level slice)."""
        self.ensure_contains(robot_x, robot_y, margin=self.max_range + 1.0)
        r0c, r0r = self.world_to_cell(robot_x, robot_y)
        h, w = self.grid.shape

        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)

        for px, py, pz in points_xyz:
            # Transform to map frame
            mx = robot_x + cos_y * px - sin_y * py
            my = robot_y + sin_y * px + cos_y * py

            # Only use floor-level points for 2D grid
            if pz < -0.1 or pz > 0.3:
                continue

            dist = math.hypot(px, py)
            if dist > self.max_range or dist < 0.1:
                continue

            ec, er = self.world_to_cell(mx, my)

            # Bresenham raycast
            cells = _bresenham(r0c, r0r, ec, er)
            for i, (cx, cy) in enumerate(cells):
                if 0 <= cx < w and 0 <= cy < h:
                    if i < len(cells) - 1:
                        self.grid[cy, cx] = max(
                            self.grid[cy, cx] + self.miss_log, self.clamp_min)
                    else:
                        self.grid[cy, cx] = min(
                            self.grid[cy, cx] + self.hit_log, self.clamp_max)

    def to_array(self):
        """Convert log-odds to int8: 100=occupied, 0=free, -1=unknown."""
        arr = np.full(self.grid.shape, -1, dtype=np.int8)
        arr[self.grid > 0.7] = 100
        arr[self.grid < -0.7] = 0
        return arr


def _bresenham(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


def parse_pointcloud2_xyz(msg):
    """Extract (x,y,z) tuples from a PointCloud2 DDS message."""
    data = bytes(msg.data)
    step = msg.point_step
    n = msg.width * msg.height
    points = []
    for i in range(n):
        offset = i * step
        x, y, z = struct.unpack_from('<fff', data, offset)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


# ===================================================================
# FrontierExplorer — main exploration logic (pure Python)
# ===================================================================
class FrontierExplorer:
    def __init__(self, network_interface="eth0"):
        print("[Explorer] Initialising SDK2...")
        ChannelFactoryInitialize(0, network_interface)

        # SLAM client
        self.slam = SlamClient()
        self.slam.SetTimeout(10.0)
        self.slam.Init()

        # Occupancy grid
        self.occ = OccupancyGrid(resolution=0.05, initial_size_m=20.0,
                                 max_range=8.0)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_valid = False
        self.navigating = False
        self.current_goal = None
        self.current_frontier = None
        self.prev_goal = None
        self.initial_pose = None
        self.blacklisted = []  # [(x, y, time)]

        # Thread-safe flags
        self._lock = threading.Lock()
        self._arrived = False
        self._nav_failed = False
        self._cloud_buffer = None  # latest point cloud

        # Config
        self.min_frontier_size = 20
        self.blacklist_radius = 0.5
        self.blacklist_timeout = 120.0
        self.progress_timeout = 30.0
        self.nav_speed = 0.8
        self.nav_mode = 0
        self.clearance_scale = 0.2
        self.map_save_path = "/home/unitree/auto_explore.pcd"

        self.last_progress_time = None
        self.last_progress_pos = None

        # DDS subscribers
        self._sub_slam_info = ChannelSubscriber("rt/slam_info", String_)
        self._sub_slam_info.Init(self._slam_info_cb, 10)

        self._sub_slam_key = ChannelSubscriber("rt/slam_key_info", String_)
        self._sub_slam_key.Init(self._slam_key_cb, 10)

        self._sub_cloud = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self._sub_cloud.Init(self._cloud_cb, 1)

        print("[Explorer] SDK2 ready")

    # ── DDS callbacks (run in SDK threads) ─────────────────────────
    def _slam_info_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if data.get("errorCode", 0) != 0:
            return

        if data.get("type") == "pos_info":
            pose = data["data"]["currentPose"]
            with self._lock:
                self.robot_x = pose["x"]
                self.robot_y = pose["y"]
                self.robot_yaw = pose.get("yaw", 0.0)
                self.pose_valid = True

        elif data.get("type") == "ctrl_info":
            ctrl = data.get("data", {})
            pose = ctrl.get("currentPose", {})
            with self._lock:
                if pose.get("x", 0.0) != 0.0 or pose.get("y", 0.0) != 0.0:
                    self.robot_x = pose["x"]
                    self.robot_y = pose["y"]
                    self.robot_yaw = pose.get("yaw", 0.0)
                    self.pose_valid = True

    def _slam_key_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("errorCode", 0) != 0:
            with self._lock:
                self._nav_failed = True
            return
        if data.get("type") == "task_result":
            arrived = data.get("data", {}).get("is_arrived", False)
            with self._lock:
                if arrived:
                    self._arrived = True
                else:
                    self._nav_failed = True

    def _cloud_cb(self, msg):
        points = parse_pointcloud2_xyz(msg)
        with self._lock:
            self._cloud_buffer = points

    # ── Main loop ──────────────────────────────────────────────────
    def run(self):
        # Start mapping
        print("[Explorer] Starting SLAM mapping...")
        code, data = self.slam.StartMapping("indoor")
        print(f"[Explorer] StartMapping: code={code}")

        # Wait for pose
        print("[Explorer] Waiting for valid pose...")
        for _ in range(60):
            with self._lock:
                if self.pose_valid:
                    break
            time.sleep(0.5)

        with self._lock:
            if not self.pose_valid:
                print("[Explorer] ERROR: No pose after 30s. Is SLAM running?")
                return

        print(f"[Explorer] Got initial pose: "
              f"({self.robot_x:.2f}, {self.robot_y:.2f})")
        self.initial_pose = (self.robot_x, self.robot_y)

        # Wait for some map data to accumulate
        print("[Explorer] Accumulating map data (5s)...")
        time.sleep(5)

        try:
            self._explore_loop()
        except KeyboardInterrupt:
            print("\n[Explorer] Stopped by user")
        finally:
            print("[Explorer] Ending mapping...")
            code, data = self.slam.EndMapping(self.map_save_path)
            print(f"[Explorer] EndMapping: code={code}, saved to "
                  f"{self.map_save_path}")

    def _explore_loop(self):
        print("[Explorer] Starting exploration loop")
        while True:
            # 1. Update grid from point cloud
            self._integrate_cloud()

            # 2. Check arrival
            self._check_arrival()

            # 3. Check progress
            self._check_progress()

            # 4. Plan if not navigating
            if not self.navigating:
                done = self._make_plan()
                if done:
                    print("[Explorer] Exploration complete!")
                    return

            time.sleep(2.0)  # plan at ~0.5 Hz

    def _integrate_cloud(self):
        with self._lock:
            cloud = self._cloud_buffer
            self._cloud_buffer = None
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        if cloud is not None and len(cloud) > 0:
            self.occ.update_from_points(rx, ry, ryaw, cloud)

    def _check_arrival(self):
        with self._lock:
            arrived = self._arrived
            failed = self._nav_failed
            self._arrived = False
            self._nav_failed = False

        if arrived and self.navigating:
            print("[Explorer] Navigation arrived!")
            if self.current_frontier:
                self._blacklist_point(*self.current_frontier)
            self.navigating = False
            self.prev_goal = None

        if failed and self.navigating:
            print("[Explorer] Navigation failed — blacklisting")
            if self.current_goal:
                self._blacklist_point(*self.current_goal)
            self.navigating = False
            self.prev_goal = None

    def _check_progress(self):
        if not self.navigating or self.last_progress_pos is None:
            return
        with self._lock:
            rx, ry = self.robot_x, self.robot_y
        dist = math.hypot(rx - self.last_progress_pos[0],
                          ry - self.last_progress_pos[1])
        if dist > 0.3:
            self.last_progress_time = time.time()
            self.last_progress_pos = (rx, ry)
            return
        elapsed = time.time() - self.last_progress_time
        if elapsed > self.progress_timeout:
            print(f"[Explorer] No progress for {elapsed:.0f}s — cancelling")
            try:
                self.slam.PauseNavigation()
            except Exception:
                pass
            if self.current_goal:
                self._blacklist_point(*self.current_goal)
            self.navigating = False

    def _make_plan(self):
        """Find frontiers and navigate to best one. Returns True if done."""
        map_array = self.occ.to_array()
        h, w = map_array.shape

        # Check if we have enough mapped area
        free_count = np.sum(map_array == 0)
        if free_count < 100:
            print("[Explorer] Not enough mapped area yet...",)
            return False

        with self._lock:
            rx, ry = self.robot_x, self.robot_y

        # Robot cell
        mx, my = self.occ.world_to_cell(rx, ry)
        if mx < 0 or mx >= w or my < 0 or my >= h:
            print("[Explorer] Robot outside grid bounds")
            return False

        if map_array[my, mx] != 0:
            found = self._nearest_free_cell(map_array, mx, my, w, h)
            if found is None:
                print("[Explorer] No free cell near robot")
                return False
            mx, my = found

        # BFS frontier search
        frontiers = self._search_frontiers(
            (rx, ry), map_array, mx, my, w, h)

        if not frontiers:
            print("[Explorer] No frontiers found — exploration complete!")
            return True

        # Filter blacklisted
        now = time.time()
        self.blacklisted = [
            (bx, by, t) for bx, by, t in self.blacklisted
            if now - t < self.blacklist_timeout
        ]
        valid = [f for f in frontiers
                 if not self._is_blacklisted(*f.centroid)]

        if not valid:
            print("[Explorer] All frontiers blacklisted — clearing")
            self.blacklisted.clear()
            valid = frontiers

        best = valid[0]
        gx, gy = best.centroid

        print(f"[Explorer] Best frontier: ({gx:.2f}, {gy:.2f}) "
              f"dist={best.min_distance:.2f}m size={best.size} "
              f"cost={best.cost:.1f}")

        # Skip too-close goals
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < 0.3:
            print("[Explorer] Goal too close — blacklisting")
            self._blacklist_point(gx, gy)
            return False

        # Same-goal detection
        if self.prev_goal is not None:
            if math.hypot(gx - self.prev_goal[0],
                          gy - self.prev_goal[1]) < 0.01:
                return False

        # Navigate
        self.current_frontier = best.centroid
        self._navigate_to(gx, gy)
        return False

    def _navigate_to(self, x, y):
        print(f"[Explorer] Navigating to ({x:.2f}, {y:.2f}) "
              f"speed={self.nav_speed}")
        code, data = self.slam.NavigateTo(
            x=x, y=y, mode=self.nav_mode, speed=self.nav_speed)

        if code != 0:
            print(f"[Explorer] NavigateTo failed: code={code}")
            self._blacklist_point(x, y)
            return

        self.current_goal = (x, y)
        self.prev_goal = (x, y)
        self.navigating = True
        self.last_progress_time = time.time()
        with self._lock:
            self.last_progress_pos = (self.robot_x, self.robot_y)

    # ── BFS frontier search ────────────────────────────────────────
    def _search_frontiers(self, robot_xy, map_array, mx, my, w, h):
        MAP_OPEN = 1
        MAP_CLOSED = 2
        FRONTIER_OPEN = 3
        FRONTIER_CLOSED = 4

        state = np.zeros((h, w), dtype=np.uint8)
        bfs_queue = deque()
        bfs_queue.append((mx, my))
        state[my, mx] = MAP_OPEN

        frontiers = []
        nbrs = [(-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)]
        res = self.occ.resolution
        ox = self.occ.origin_x
        oy = self.occ.origin_y

        while bfs_queue:
            cx, cy = bfs_queue.popleft()
            if state[cy, cx] == MAP_CLOSED:
                continue
            state[cy, cx] = MAP_CLOSED

            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue

                if state[ny, nx] not in (FRONTIER_OPEN, FRONTIER_CLOSED):
                    if self._is_frontier_cell(map_array, nx, ny, w, h, nbrs):
                        frontier = self._build_frontier(
                            map_array, state, nx, ny, w, h,
                            robot_xy, ox, oy, res, nbrs,
                            FRONTIER_OPEN, FRONTIER_CLOSED)
                        if frontier.size >= self.min_frontier_size:
                            frontiers.append(frontier)

                val = map_array[ny, nx]
                if val == 0 and state[ny, nx] not in (MAP_OPEN, MAP_CLOSED):
                    state[ny, nx] = MAP_OPEN
                    bfs_queue.append((nx, ny))

        # Cost = nearest first
        for f in frontiers:
            f.cost = f.min_distance
        frontiers.sort(key=lambda f: f.cost)
        return frontiers

    @staticmethod
    def _is_frontier_cell(map_array, x, y, w, h, nbrs):
        if map_array[y, x] != -1:
            return False
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                if map_array[ny, nx] == 0:
                    return True
        return False

    def _build_frontier(self, map_array, state, sx, sy, w, h,
                        robot_xy, ox, oy, res, nbrs,
                        FRONTIER_OPEN, FRONTIER_CLOSED):
        frontier = Frontier()
        fqueue = deque()
        fqueue.append((sx, sy))
        state[sy, sx] = FRONTIER_OPEN
        sum_x = sum_y = 0.0

        while fqueue:
            cx, cy = fqueue.popleft()
            if state[cy, cx] == FRONTIER_CLOSED:
                continue
            state[cy, cx] = FRONTIER_CLOSED

            wx = cx * res + ox
            wy = cy * res + oy
            d = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if d < frontier.min_distance:
                frontier.min_distance = d
            sum_x += wx
            sum_y += wy
            frontier.points.append((wx, wy))
            frontier.size += 1

            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if state[ny, nx] not in (FRONTIER_OPEN, FRONTIER_CLOSED):
                    if self._is_frontier_cell(map_array, nx, ny, w, h, nbrs):
                        state[ny, nx] = FRONTIER_OPEN
                        fqueue.append((nx, ny))

        if frontier.size > 0:
            frontier.centroid = (sum_x / frontier.size,
                                sum_y / frontier.size)
        return frontier

    @staticmethod
    def _nearest_free_cell(map_array, sx, sy, w, h, max_radius=50):
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    nx, ny = sx + dx, sy + dy
                    if 0 <= nx < w and 0 <= ny < h and map_array[ny, nx] == 0:
                        return (nx, ny)
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    nx, ny = sx + dx, sy + dy
                    if 0 <= nx < w and 0 <= ny < h and map_array[ny, nx] == 0:
                        return (nx, ny)
        return None

    # ── Blacklisting ───────────────────────────────────────────────
    def _blacklist_point(self, x, y):
        print(f"[Explorer] Blacklisting ({x:.2f}, {y:.2f})")
        self.blacklisted.append((x, y, time.time()))

    def _is_blacklisted(self, x, y):
        for bx, by, _ in self.blacklisted:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False


# ===================================================================
# Entry point
# ===================================================================
def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    print(f"[Explorer] Using network interface: {iface}")

    explorer = FrontierExplorer(network_interface=iface)
    explorer.run()


if __name__ == "__main__":
    main()
