#!/usr/bin/env python3
"""
Frontier-based autonomous explorer for GO2W using Unitree SLAM API.

Forked from auto_explore_sim/scripts/frontier_explorer.py.
Nav2 action clients replaced with Unitree SLAM service calls (API 1102).
Arrival detection via ROS2 ``/slam_key_info`` topic (std_msgs/String).

Key features:
  - BFS frontier search outward from robot position
  - min_distance per frontier (closest cell to robot, not centroid)
  - Nearest-first cost with GVD clearance bonus for tiebreaking
  - Same-goal detection (skip re-sending identical goal)
  - Stop/Resume via explore/resume topic (std_msgs/Bool)
  - Return-to-init option when exploration completes
  - Blacklisting of unreachable frontiers with timeout
  - RViz frontier marker visualisation
  - Mission lifecycle: auto-start mapping, auto-end mapping
"""

import json
import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

# Local SLAM client (pure ROS2 pub/sub, same scripts/ directory)
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slam_client import SlamClient


# ---------------------------------------------------------------------------
# Frontier data container
# ---------------------------------------------------------------------------
class Frontier:
    """One frontier cluster discovered by BFS."""
    __slots__ = ('size', 'min_distance', 'cost', 'centroid', 'middle',
                 'initial', 'points')

    def __init__(self):
        self.size = 0
        self.min_distance = float('inf')
        self.cost = 0.0
        self.centroid = (0.0, 0.0)
        self.middle = (0.0, 0.0)
        self.initial = (0.0, 0.0)
        self.points = []


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class FrontierExplorerNode(Node):
    """Autonomous frontier exploration using Unitree SLAM navigation."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self.get_logger().info('Frontier Explorer (Unitree SLAM) starting...')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('planner_frequency', 0.5)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('transform_tolerance', 2.0)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('blacklist_timeout', 60.0)
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('clearance_scale', 0.3)
        self.declare_parameter('return_to_init', False)
        self.declare_parameter('gvd_min_clearance', 3)
        self.declare_parameter('gvd_snap_radius', 2.0)
        self.declare_parameter('visualize_gvd', True)
        # Unitree-specific
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('nav_speed', 0.8)
        self.declare_parameter('nav_mode', 0)  # 0=avoid, 1=stop
        self.declare_parameter('auto_start_mapping', True)
        self.declare_parameter('map_save_path',
                               '/home/unitree/auto_explore.pcd')

        # ── Read parameters ─────────────────────────────────────────────
        self.planner_freq = self.get_parameter('planner_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.tf_tolerance = self.get_parameter('transform_tolerance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_timeout = self.get_parameter('blacklist_timeout').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.visualize = self.get_parameter('visualize').value
        self.clearance_scale = self.get_parameter('clearance_scale').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.gvd_min_clearance = self.get_parameter('gvd_min_clearance').value
        self.gvd_snap_radius = self.get_parameter('gvd_snap_radius').value
        self.visualize_gvd = self.get_parameter('visualize_gvd').value
        self.network_interface = self.get_parameter('network_interface').value
        self.nav_speed = self.get_parameter('nav_speed').value
        self.nav_mode = self.get_parameter('nav_mode').value
        self.auto_start_mapping = self.get_parameter('auto_start_mapping').value
        self.map_save_path = self.get_parameter('map_save_path').value

        # ── SLAM client (pure ROS2 pub/sub, no SDK2) ─────────────────
        self.slam_client = SlamClient(self, timeout=10.0)
        self.get_logger().info('SlamClient initialised (pure ROS2)')

        # ROS2 subscriber for arrival feedback (was SDK2 DDS)
        self._arrival_lock = threading.Lock()
        self._arrived = False
        self._nav_failed = False
        self._slam_key_sub = self.create_subscription(
            String, '/slam_key_info', self._slam_key_info_cb, 10)

        # Auto-start mapping (delayed to let pub/sub connect)
        if self.auto_start_mapping:
            self._mapping_started = False
            self.create_timer(3.0, self._try_start_mapping)

        # ── TF listener ────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ────────────────────────────────────────────────
        self.map_data = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        self.exploring = True
        self.resume_sub = self.create_subscription(
            Bool, 'explore/resume', self._resume_callback, 10)

        # ── Visualisation publishers ───────────────────────────────────
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'explore/frontiers', 10)
        if self.visualize_gvd:
            self.gvd_marker_pub = self.create_publisher(
                MarkerArray, 'explore/gvd', 10)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, 'explore/gvd_path', 10)

        # ── State ──────────────────────────────────────────────────────
        self.navigating = False
        self.current_goal = None
        self.current_frontier = None
        self.prev_goal = None
        self.blacklisted = []
        self.last_progress_time = None
        self.last_robot_pos = None
        self.initial_pose = None
        self._cached_dist_map = None
        self._cached_map_stamp = None
        self._cached_label_map = None
        self._cached_gvd_mask = None

        # Backoff state for transient SLAM errors (e.g. errorCode=4)
        self._consecutive_nav_failures = 0
        self._nav_backoff_until = None  # Time object; skip planning until then
        self._slam_ready = False  # True once first NavigateTo succeeds

        # ── Timer ──────────────────────────────────────────────────────
        period = 1.0 / max(self.planner_freq, 0.01)
        self.timer = self.create_timer(period, self._explore_tick)

        self.get_logger().info(
            f'Frontier Explorer ready  (freq={self.planner_freq} Hz, '
            f'min_frontier={self.min_frontier_size} cells, '
            f'nav_speed={self.nav_speed}, nav_mode={self.nav_mode})')

    def destroy_node(self):
        """Clean up SLAM on shutdown."""
        if self.auto_start_mapping:
            self.get_logger().info('Ending SLAM mapping on shutdown...')
            try:
                self.slam_client.EndMapping(self.map_save_path)
            except Exception as e:
                self.get_logger().warn(f'EndMapping failed: {e}')
        super().destroy_node()

    # ================================================================
    # Arrival callback (ROS2 subscription on /slam_key_info)
    # ================================================================

    def _try_start_mapping(self):
        """Delayed mapping start to let pub/sub connect."""
        if self._mapping_started:
            return
        self.get_logger().info('Auto-starting SLAM mapping...')
        code, data = self.slam_client.StartMapping("indoor")
        self.get_logger().info(
            f'StartMapping response: code={code}, data={data}')
        if code == 0:
            self._mapping_started = True
            # Give SLAM time to converge before allowing navigation
            self._nav_backoff_until = (
                self.get_clock().now() + Duration(seconds=10.0))
            self.get_logger().info(
                'Mapping started — waiting 10s for SLAM to converge')
        elif code == -1:
            self.get_logger().warning(
                'StartMapping timed out — will retry in 3s')

    def _slam_key_info_cb(self, msg: String):
        """Called on /slam_key_info ROS2 topic."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        self.get_logger().debug(f'slam_key_info: {msg.data}')

        error_code = data.get("errorCode", 0)
        if error_code != 0:
            self.get_logger().warning(
                f'slam_key_info errorCode={error_code}')
            with self._arrival_lock:
                self._nav_failed = True
            return

        if data.get("type") == "task_result":
            arrived = data.get("data", {}).get("is_arrived", False)
            self.get_logger().info(
                f'slam_key_info task_result: is_arrived={arrived}')
            with self._arrival_lock:
                if arrived:
                    self._arrived = True
                else:
                    self._nav_failed = True

    # ================================================================
    # Callbacks
    # ================================================================

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def _resume_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Exploration RESUMED')
            self.exploring = True
        else:
            self.get_logger().info('Exploration STOPPED')
            self.exploring = False
            self._cancel_current_goal()
            self.navigating = False

    # ================================================================
    # Main exploration loop
    # ================================================================

    def _explore_tick(self):
        if not self.exploring:
            return

        # Check arrival from DDS feedback
        self._check_arrival()

        self._make_plan()

    def _check_arrival(self):
        """Check thread-safe arrival/failure flags set by DDS callback."""
        with self._arrival_lock:
            arrived = self._arrived
            failed = self._nav_failed
            self._arrived = False
            self._nav_failed = False

        if arrived and self.navigating:
            self.get_logger().info('Navigation arrived (SLAM feedback)')
            if self.current_frontier is not None:
                self._blacklist_point(
                    self.current_frontier[0], self.current_frontier[1])
            self.navigating = False
            self.prev_goal = None

        if failed and self.navigating:
            self.get_logger().warning(
                'Navigation failed (SLAM feedback) — blacklisting')
            self._blacklist_current_goal()
            self.navigating = False
            self.prev_goal = None

    def _make_plan(self):
        """Core planning: find frontiers from robot, pick best, navigate."""
        if self.map_data is None:
            self.get_logger().info(
                'Waiting for map...', throttle_duration_sec=5.0)
            return

        # Backoff after transient SLAM failures
        if self._nav_backoff_until is not None:
            remaining = (
                self._nav_backoff_until - self.get_clock().now()
            ).nanoseconds / 1e9
            if remaining > 0:
                self.get_logger().info(
                    f'SLAM backoff: {remaining:.0f}s remaining',
                    throttle_duration_sec=5.0)
                return
            self._nav_backoff_until = None

        robot_xy = self._get_robot_position()
        if robot_xy is None:
            return

        if self.initial_pose is None:
            self.initial_pose = robot_xy
            self.get_logger().info(
                f'Initial pose stored: '
                f'({robot_xy[0]:.2f}, {robot_xy[1]:.2f})')

        self._check_progress(robot_xy)

        if self.navigating:
            return

        # BFS frontier search
        info = self.map_data.info
        map_array = np.array(self.map_data.data, dtype=np.int8).reshape(
            (info.height, info.width))

        frontiers = self._search_from(robot_xy, map_array, info)

        if not frontiers:
            self.get_logger().info(
                'No frontiers found — exploration complete!',
                throttle_duration_sec=10.0)
            if self.auto_start_mapping:
                self.get_logger().info(
                    f'Ending mapping, saving to {self.map_save_path}')
                code, data = self.slam_client.EndMapping(self.map_save_path)
                self.get_logger().info(
                    f'EndMapping: code={code}, data={data}')
                self.auto_start_mapping = False  # only once
            if self.return_to_init and self.initial_pose is not None:
                self._return_to_initial_pose()
            return

        # Filter blacklisted
        now = self.get_clock().now()
        self.blacklisted = [
            (bx, by, t) for bx, by, t in self.blacklisted
            if (now - t).nanoseconds / 1e9 < self.blacklist_timeout
        ]
        valid = [f for f in frontiers if not self._is_blacklisted(*f.centroid)]

        if not valid:
            self.get_logger().warn(
                'All frontiers blacklisted — clearing blacklist')
            self.blacklisted.clear()
            valid = frontiers

        best = valid[0]

        self.get_logger().info(
            f'Best frontier: ({best.centroid[0]:.2f}, {best.centroid[1]:.2f})  '
            f'min_dist={best.min_distance:.2f}m  size={best.size}  '
            f'cost={best.cost:.1f}')

        if self.visualize:
            self._publish_markers(valid, best)
        if self.visualize_gvd:
            self._publish_gvd_markers(map_array, info)

        # GVD snap (single goal — Unitree SLAM has its own planner)
        gx, gy = self._snap_to_gvd(
            best.centroid, robot_xy, map_array, info)

        # Skip if goal is too close (instant-success loop)
        dist_to_goal = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if dist_to_goal < 0.3:
            self.get_logger().warning(
                f'Goal ({gx:.2f}, {gy:.2f}) only {dist_to_goal:.2f}m away '
                f'— blacklisting frontier')
            self._blacklist_point(best.centroid[0], best.centroid[1])
            return

        # Same-goal detection
        if self.prev_goal is not None:
            dx = gx - self.prev_goal[0]
            dy = gy - self.prev_goal[1]
            if math.sqrt(dx * dx + dy * dy) < 0.01:
                self.get_logger().debug('Same goal as before — skipping')
                return

        # Navigate via Unitree SLAM
        self.current_frontier = best.centroid
        self._navigate_to(gx, gy)

    # ================================================================
    # BFS frontier search from robot position
    # ================================================================

    def _search_from(self, robot_xy, map_array, info):
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        mx = int((robot_xy[0] - ox) / resolution)
        my = int((robot_xy[1] - oy) / resolution)

        if mx < 0 or mx >= w or my < 0 or my >= h:
            self.get_logger().warning('Robot position outside map bounds')
            return []

        if map_array[my, mx] != 0:
            found = self._nearest_free_cell(map_array, mx, my, w, h)
            if found is None:
                self.get_logger().warning('Cannot find free cell near robot')
                return []
            mx, my = found

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
                            robot_xy, ox, oy, resolution, nbrs,
                            FRONTIER_OPEN, FRONTIER_CLOSED)
                        if frontier.size >= self.min_frontier_size:
                            frontiers.append(frontier)

                val = map_array[ny, nx]
                if val == 0 and state[ny, nx] not in (MAP_OPEN, MAP_CLOSED):
                    state[ny, nx] = MAP_OPEN
                    bfs_queue.append((nx, ny))

        dist_map = self._get_distance_transform(map_array)

        for f in frontiers:
            cx = int((f.centroid[0] - ox) / resolution)
            cy = int((f.centroid[1] - oy) / resolution)
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))
            clearance = float(dist_map[cy, cx]) * resolution
            f.cost = f.min_distance - self.clearance_scale * clearance

        frontiers.sort(key=lambda f: f.cost)
        return frontiers

    def _is_frontier_cell(self, map_array, x, y, w, h, nbrs):
        if map_array[y, x] != -1:
            return False
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                if map_array[ny, nx] == 0:
                    return True
        return False

    def _build_frontier(self, map_array, state, sx, sy, w, h,
                        robot_xy, ox, oy, resolution, nbrs,
                        FRONTIER_OPEN, FRONTIER_CLOSED):
        frontier = Frontier()
        fqueue = deque()
        fqueue.append((sx, sy))
        state[sy, sx] = FRONTIER_OPEN

        sum_x = 0.0
        sum_y = 0.0

        while fqueue:
            cx, cy = fqueue.popleft()
            if state[cy, cx] == FRONTIER_CLOSED:
                continue
            state[cy, cx] = FRONTIER_CLOSED

            wx = cx * resolution + ox
            wy = cy * resolution + oy

            d = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if d < frontier.min_distance:
                frontier.min_distance = d

            sum_x += wx
            sum_y += wy
            frontier.points.append((wx, wy))

            if frontier.size == 0:
                frontier.initial = (wx, wy)

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
            frontier.centroid = (sum_x / frontier.size, sum_y / frontier.size)
            mid_idx = frontier.size // 2
            frontier.middle = frontier.points[mid_idx]

        return frontier

    def _nearest_free_cell(self, map_array, sx, sy, w, h, max_radius=50):
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

    # ================================================================
    # GVD distance transform & Voronoi skeleton
    # ================================================================

    def _get_distance_transform(self, map_array):
        stamp = self.map_data.header.stamp if self.map_data else None
        if (self._cached_dist_map is not None
                and self._cached_map_stamp == stamp
                and self._cached_dist_map.shape == map_array.shape):
            return self._cached_dist_map

        self._cached_dist_map, self._cached_label_map = \
            self._compute_distance_and_labels(map_array)
        self._cached_gvd_mask = self._extract_gvd_mask(
            self._cached_dist_map, self._cached_label_map, map_array)
        self._cached_map_stamp = stamp
        return self._cached_dist_map

    def _get_gvd_mask(self, map_array):
        self._get_distance_transform(map_array)
        return self._cached_gvd_mask

    def _compute_distance_and_labels(self, map_array):
        h, w = map_array.shape
        obstacle = map_array > 50

        region_id = np.full((h, w), -1, dtype=np.int32)
        current_label = 0

        for sy in range(h):
            for sx in range(w):
                if obstacle[sy, sx] and region_id[sy, sx] == -1:
                    cc_queue = deque()
                    cc_queue.append((sx, sy))
                    region_id[sy, sx] = current_label
                    while cc_queue:
                        cx, cy = cc_queue.popleft()
                        for ddx in (-1, 0, 1):
                            for ddy in (-1, 0, 1):
                                if ddx == 0 and ddy == 0:
                                    continue
                                nx, ny = cx + ddx, cy + ddy
                                if (0 <= nx < w and 0 <= ny < h
                                        and obstacle[ny, nx]
                                        and region_id[ny, nx] == -1):
                                    region_id[ny, nx] = current_label
                                    cc_queue.append((nx, ny))
                    current_label += 1

        self.get_logger().debug(
            f'GVD: found {current_label} obstacle regions',
            throttle_duration_sec=10.0)

        dist = np.full((h, w), -1, dtype=np.int32)
        label = np.full((h, w), -1, dtype=np.int32)
        queue = deque()

        obs_y, obs_x = np.where(obstacle)
        for i in range(len(obs_y)):
            y, x = int(obs_y[i]), int(obs_x[i])
            dist[y, x] = 0
            label[y, x] = region_id[y, x]
            queue.append((x, y))

        while queue:
            cx, cy = queue.popleft()
            nd = dist[cy, cx] + 1
            lbl = label[cy, cx]
            for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + ddx, cy + ddy
                if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] == -1:
                    dist[ny, nx] = nd
                    label[ny, nx] = lbl
                    queue.append((nx, ny))

        dist[dist < 0] = 0
        return dist, label

    def _extract_gvd_mask(self, dist_map, label_map, map_array):
        h, w = dist_map.shape
        min_c = self.gvd_min_clearance
        gvd = np.zeros((h, w), dtype=bool)

        for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if ddx == 1:
                src = label_map[:, :-1]
                nbr = label_map[:, 1:]
                dst = gvd[:, :-1]
            elif ddx == -1:
                src = label_map[:, 1:]
                nbr = label_map[:, :-1]
                dst = gvd[:, 1:]
            elif ddy == 1:
                src = label_map[:-1, :]
                nbr = label_map[1:, :]
                dst = gvd[:-1, :]
            else:
                src = label_map[1:, :]
                nbr = label_map[:-1, :]
                dst = gvd[1:, :]
            diff = (src != nbr) & (src >= 0) & (nbr >= 0)
            dst |= diff

        gvd &= (map_array == 0)
        gvd &= (dist_map >= min_c)
        return gvd

    def _snap_to_gvd(self, point, robot_xy, map_array, info):
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return point

        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height

        px = int((point[0] - ox) / resolution)
        py = int((point[1] - oy) / resolution)
        px = max(0, min(w - 1, px))
        py = max(0, min(h - 1, py))

        if gvd_mask[py, px]:
            return point

        robot_to_frontier = math.hypot(
            point[0] - robot_xy[0], point[1] - robot_xy[1])

        max_cells = int(self.gvd_snap_radius / resolution)
        visited = set()
        queue = deque()
        queue.append((px, py))
        visited.add((px, py))

        best_gvd = None
        best_dist = float('inf')

        while queue:
            cx, cy = queue.popleft()
            if gvd_mask[cy, cx]:
                wx = cx * resolution + ox
                wy = cy * resolution + oy
                cand_to_frontier = math.hypot(wx - point[0], wy - point[1])
                if cand_to_frontier < best_dist:
                    cand_to_robot = math.hypot(
                        wx - robot_xy[0], wy - robot_xy[1])
                    if cand_to_robot < robot_to_frontier:
                        best_gvd = (wx, wy)
                        best_dist = cand_to_frontier

            for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + ddx, cy + ddy
                if (0 <= nx < w and 0 <= ny < h
                        and (nx, ny) not in visited
                        and abs(nx - px) <= max_cells
                        and abs(ny - py) <= max_cells):
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        if best_gvd is not None:
            self.get_logger().info(
                f'Frontier snapped to GVD: '
                f'({point[0]:.2f}, {point[1]:.2f}) -> '
                f'({best_gvd[0]:.2f}, {best_gvd[1]:.2f})')
            return best_gvd

        self.get_logger().debug(
            f'No valid GVD point within {self.gvd_snap_radius}m '
            f'— using original centroid')
        return point

    # ================================================================
    # Navigation (Unitree SLAM API 1102)
    # ================================================================

    def _navigate_to(self, x: float, y: float):
        """Send a navigation goal via Unitree SLAM API 1102."""
        self.get_logger().info(
            f'Sending goal via SLAM API: ({x:.2f}, {y:.2f}) '
            f'speed={self.nav_speed} mode={self.nav_mode}')

        code, data = self.slam_client.NavigateTo(
            x=x, y=y, z=0.0,
            q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0,
            mode=self.nav_mode, speed=self.nav_speed,
        )

        if code != 0:
            self.get_logger().error(
                f'SLAM NavigateTo failed: code={code}, data={data}')

            # Check if this is a transient SLAM error (not goal-specific)
            # errorCode=4 means "Failed to obtain current pose" — SLAM not ready
            is_transient = False
            try:
                resp = json.loads(data) if isinstance(data, str) and data else {}
                error_code = resp.get('errorCode', 0)
                if error_code == 4:
                    is_transient = True
            except (json.JSONDecodeError, TypeError):
                pass

            if is_transient or code == -1:  # -1 = timeout
                # Don't blacklist — SLAM isn't ready, not a bad goal
                self._consecutive_nav_failures += 1
                backoff_secs = min(
                    30.0, 3.0 * (2 ** (self._consecutive_nav_failures - 1)))
                self._nav_backoff_until = (
                    self.get_clock().now()
                    + Duration(seconds=backoff_secs))
                self.get_logger().warning(
                    f'Transient SLAM error (attempt {self._consecutive_nav_failures})'
                    f' — backing off {backoff_secs:.0f}s before retry')
                self.navigating = False
                return

            # Goal-specific failure — blacklist this goal
            self._consecutive_nav_failures = 0
            self._blacklist_current_goal()
            self.navigating = False
            return

        # Success
        self.get_logger().info(f'Goal accepted (code={code})')
        self._slam_ready = True
        self._consecutive_nav_failures = 0
        self._nav_backoff_until = None
        self.current_goal = (x, y)
        self.prev_goal = (x, y)
        self.navigating = True
        self.last_progress_time = self.get_clock().now()
        self.last_robot_pos = self._get_robot_position()

    def _return_to_initial_pose(self):
        if self.initial_pose is None or self.navigating:
            return
        self.get_logger().info(
            f'Exploration complete — returning to initial pose '
            f'({self.initial_pose[0]:.2f}, {self.initial_pose[1]:.2f})')
        self.exploring = False
        self._navigate_to(self.initial_pose[0], self.initial_pose[1])

    # ================================================================
    # Progress checking
    # ================================================================

    def _check_progress(self, robot_xy):
        if not self.navigating or self.last_robot_pos is None:
            return

        dist_moved = math.hypot(
            robot_xy[0] - self.last_robot_pos[0],
            robot_xy[1] - self.last_robot_pos[1])

        if dist_moved > 0.3:
            self.last_progress_time = self.get_clock().now()
            self.last_robot_pos = robot_xy
            return

        elapsed = (
            self.get_clock().now() - self.last_progress_time
        ).nanoseconds / 1e9
        if elapsed > self.progress_timeout:
            self.get_logger().warning(
                f'No progress for {elapsed:.0f}s — cancelling goal')
            self._cancel_current_goal()
            self._blacklist_current_goal()
            self.navigating = False

    def _cancel_current_goal(self):
        """Pause Unitree SLAM navigation (equivalent to Nav2 cancel)."""
        self.get_logger().info('Pausing SLAM navigation...')
        try:
            self.slam_client.PauseNavigation()
        except Exception as e:
            self.get_logger().warn(f'PauseNavigation failed: {e}')

    # ================================================================
    # Blacklisting
    # ================================================================

    def _blacklist_current_goal(self):
        if self.current_goal is not None:
            self._blacklist_point(self.current_goal[0], self.current_goal[1])

    def _blacklist_point(self, x, y):
        self.get_logger().info(f'Blacklisting ({x:.2f}, {y:.2f})')
        self.blacklisted.append((x, y, self.get_clock().now()))

    def _is_blacklisted(self, x, y):
        for bx, by, _ in self.blacklisted:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    # ================================================================
    # TF helper
    # ================================================================

    def _get_robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_tolerance))
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    # ================================================================
    # Visualisation
    # ================================================================

    def _publish_gvd_markers(self, map_array, info):
        gvd_mask = self._get_gvd_mask(map_array)
        if gvd_mask is None:
            return

        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'gvd'
        ma.markers.append(delete_marker)

        line_marker = Marker()
        line_marker.header.frame_id = self.global_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'gvd'
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 1.0
        line_marker.color.a = 0.8
        line_marker.lifetime.sec = 10
        line_marker.pose.orientation.w = 1.0

        gvd_ys, gvd_xs = np.where(gvd_mask)
        gvd_set = set(zip(gvd_xs.tolist(), gvd_ys.tolist()))

        for x, y in gvd_set:
            wx = x * resolution + ox
            wy = y * resolution + oy
            p1 = Point(x=wx, y=wy, z=0.05)
            if (x + 1, y) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox, y=wy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            if (x, y + 1) in gvd_set:
                p2 = Point(x=wx, y=(y + 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            if (x + 1, y + 1) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox,
                           y=(y + 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            if (x + 1, y - 1) in gvd_set:
                p2 = Point(x=(x + 1) * resolution + ox,
                           y=(y - 1) * resolution + oy, z=0.05)
                line_marker.points.append(p1)
                line_marker.points.append(p2)

        if line_marker.points:
            ma.markers.append(line_marker)

        self.gvd_marker_pub.publish(ma)

    def _publish_markers(self, frontiers, chosen):
        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.centroid[0]
            m.pose.position.y = f.centroid[1]
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            scale = max(0.15, min(0.6, f.size * 0.005))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            is_chosen = (f is chosen)
            if is_chosen:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                m.color.r = 0.2
                m.color.g = 0.4
                m.color.b = 1.0
                m.color.a = 0.8

            m.lifetime.sec = 10
            ma.markers.append(m)

        self.marker_pub.publish(ma)


# ====================================================================
# Entry point
# ====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    # MultiThreadedExecutor so SlamClient response callback can fire
    # while timer callbacks block on _call()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        node.get_logger().info('Starting frontier exploration...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Exploration stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
