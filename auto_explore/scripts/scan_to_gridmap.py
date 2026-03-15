#!/usr/bin/env python3
"""Lightweight 2D occupancy-grid builder from laser scans + known pose.

Subscribes to ``/scan`` (LaserScan) and looks up the ``map → lidar`` TF
(provided by ``official_slam_odom_bridge``).  Each scan is raycasted into a
log-odds grid using Bresenham lines.  The resulting ``/map``
(OccupancyGrid) is published at a configurable rate.

No dependency on SLAM Toolbox, Nav2, or octomap – pure rclpy + numpy.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import tf2_ros
from tf2_ros import TransformException


class ScanToGridmap(Node):
    def __init__(self):
        super().__init__("scan_to_gridmap")

        # Parameters
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("initial_size_m", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lidar_frame", "lidar")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("hit_log_odds", 0.9)
        self.declare_parameter("miss_log_odds", -0.4)
        self.declare_parameter("clamp_min", -5.0)
        self.declare_parameter("clamp_max", 5.0)
        self.declare_parameter("occ_threshold", 0.7)
        self.declare_parameter("free_threshold", -0.7)

        self.resolution = self.get_parameter("resolution").value
        initial_size_m = self.get_parameter("initial_size_m").value
        self.map_frame = self.get_parameter("map_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        publish_rate = self.get_parameter("publish_rate").value
        self.max_range = self.get_parameter("max_range").value
        self.hit_log = self.get_parameter("hit_log_odds").value
        self.miss_log = self.get_parameter("miss_log_odds").value
        self.clamp_min = self.get_parameter("clamp_min").value
        self.clamp_max = self.get_parameter("clamp_max").value
        self.occ_thresh = self.get_parameter("occ_threshold").value
        self.free_thresh = self.get_parameter("free_threshold").value

        # Grid state – origin is the bottom-left corner in world coords.
        initial_cells = int(initial_size_m / self.resolution)
        self.grid = np.zeros((initial_cells, initial_cells), dtype=np.float32)
        self.origin_x = -initial_size_m / 2.0  # world x of cell (0,0)
        self.origin_y = -initial_size_m / 2.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions / publications
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, 10
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)

        self.publish_timer = self.create_timer(1.0 / publish_rate, self._publish_map)

        self.get_logger().info(
            f"scan_to_gridmap: {initial_cells}x{initial_cells} cells, "
            f"res={self.resolution}m, max_range={self.max_range}m"
        )

    # ------------------------------------------------------------------
    # Scan callback
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.lidar_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException:
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        # Yaw from quaternion
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        # Ensure robot position is within grid (expand if needed)
        self._ensure_contains(rx, ry, margin=self.max_range + 1.0)

        # Robot cell
        r0c, r0r = self._world_to_cell(rx, ry)

        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isfinite(r) or r < msg.range_min:
                angle += msg.angle_increment
                continue

            clamped = min(r, self.max_range)
            beam_angle = yaw + angle

            # Endpoint in world
            ex = rx + clamped * math.cos(beam_angle)
            ey = ry + clamped * math.sin(beam_angle)
            ec, er = self._world_to_cell(ex, ey)

            # Bresenham raycast
            cells = self._bresenham(r0c, r0r, ec, er)
            h, w = self.grid.shape
            for i, (cx, cy) in enumerate(cells):
                if 0 <= cx < w and 0 <= cy < h:
                    if i < len(cells) - 1:
                        # Free ray
                        self.grid[cy, cx] = max(
                            self.grid[cy, cx] + self.miss_log, self.clamp_min
                        )
                    elif r <= self.max_range:
                        # Hit (only if range was not clamped)
                        self.grid[cy, cx] = min(
                            self.grid[cy, cx] + self.hit_log, self.clamp_max
                        )

            angle += msg.angle_increment

    # ------------------------------------------------------------------
    # Publish OccupancyGrid
    # ------------------------------------------------------------------
    def _publish_map(self):
        h, w = self.grid.shape
        data = np.full(h * w, -1, dtype=np.int8)

        flat = self.grid.ravel()
        data[flat > self.occ_thresh] = 100
        data[flat < self.free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        info = MapMetaData()
        info.resolution = self.resolution
        info.width = w
        info.height = h
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        msg.info = info

        msg.data = data.tolist()
        self.map_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _world_to_cell(self, wx, wy):
        cx = int((wx - self.origin_x) / self.resolution)
        cy = int((wy - self.origin_y) / self.resolution)
        return cx, cy

    def _ensure_contains(self, wx, wy, margin=0.0):
        """Expand the grid if (wx, wy) +/- margin falls outside."""
        need_x_min = wx - margin
        need_x_max = wx + margin
        need_y_min = wy - margin
        need_y_max = wy + margin

        cur_x_max = self.origin_x + self.grid.shape[1] * self.resolution
        cur_y_max = self.origin_y + self.grid.shape[0] * self.resolution

        if (
            need_x_min >= self.origin_x
            and need_x_max <= cur_x_max
            and need_y_min >= self.origin_y
            and need_y_max <= cur_y_max
        ):
            return  # fits

        # New bounds (add 20% padding)
        pad = margin * 0.2
        new_x_min = min(self.origin_x, need_x_min - pad)
        new_y_min = min(self.origin_y, need_y_min - pad)
        new_x_max = max(cur_x_max, need_x_max + pad)
        new_y_max = max(cur_y_max, need_y_max + pad)

        new_w = int(math.ceil((new_x_max - new_x_min) / self.resolution))
        new_h = int(math.ceil((new_y_max - new_y_min) / self.resolution))

        new_grid = np.zeros((new_h, new_w), dtype=np.float32)

        # Copy old data into new grid
        off_c = int(round((self.origin_x - new_x_min) / self.resolution))
        off_r = int(round((self.origin_y - new_y_min) / self.resolution))
        old_h, old_w = self.grid.shape
        new_grid[off_r : off_r + old_h, off_c : off_c + old_w] = self.grid

        self.grid = new_grid
        self.origin_x = new_x_min
        self.origin_y = new_y_min

        self.get_logger().info(
            f"Grid expanded to {new_w}x{new_h} cells "
            f"({new_w * self.resolution:.1f}x{new_h * self.resolution:.1f}m)"
        )

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Integer Bresenham line from (x0,y0) to (x1,y1)."""
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


def main(args=None):
    rclpy.init(args=args)
    node = ScanToGridmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
