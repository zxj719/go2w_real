#!/usr/bin/env python3
"""
Record named navigation waypoints from RViz Publish Point.

Workflow:
  1. Start SLAM and drive the robot to build the map.
  2. Run this script.
  3. In RViz, use "Publish Point" to click waypoint positions.
  4. The script records the clicked position.
  5. The script reads the robot's current heading from TF.
  6. Enter the waypoint name in the terminal.
  7. Press Ctrl+C when finished. The script writes waypoint YAML only.
"""

import math
import os
import queue
import sys
import threading


DEFAULT_WAYPOINT_FILE = "/home/unitree/ros_ws/src/go2w_waypoints.yaml"
DEFAULT_CLICKED_POINT_TOPIC = "/clicked_point"
DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base"


def _print_help():
    print(
        "Usage: ros2 run go2w_real save_map_and_waypoints.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        "  --waypoint-file PATH       Waypoint YAML path, default: /home/unitree/ros_ws/src/go2w_waypoints.yaml\n"
        "  --clicked-point-topic TOPIC\n"
        "                             RViz Publish Point topic, default: /clicked_point\n"
        "  --map-frame FRAME          Map frame for saved waypoints, default: map\n"
        "  --base-frame FRAME         Robot base frame used for default yaw, default: base\n"
        "  -h, --help                 Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "waypoint_file": DEFAULT_WAYPOINT_FILE,
        "clicked_point_topic": DEFAULT_CLICKED_POINT_TOPIC,
        "map_frame": DEFAULT_MAP_FRAME,
        "base_frame": DEFAULT_BASE_FRAME,
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
        if arg == "--clicked-point-topic" and i + 1 < len(raw_args):
            config["clicked_point_topic"] = raw_args[i + 1]
            i += 2
            continue
        if arg.startswith("--clicked-point-topic="):
            config["clicked_point_topic"] = arg.split("=", 1)[1]
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


def _yaml_quote(text):
    escaped = str(text).replace("'", "''")
    return f"'{escaped}'"


class MapWaypointRecorder:
    def __init__(self, node, config):
        from geometry_msgs.msg import PointStamped
        import tf2_ros

        self.node = node
        self.waypoint_file = config["waypoint_file"]
        self.clicked_point_topic = config["clicked_point_topic"]
        self.map_frame = config["map_frame"]
        self.base_frame = config["base_frame"]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.node, spin_thread=False
        )

        self.pending_points = queue.Queue()
        self.waypoints = []
        self.waypoint_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.clicked_sub = self.node.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self._clicked_point_cb,
            10,
        )

        self.prompt_thread = threading.Thread(
            target=self._prompt_loop,
            name="waypoint_prompt",
            daemon=True,
        )
        self.prompt_thread.start()

        self.node.get_logger().info(
            "Waypoint recorder ready. Use RViz Publish Point on "
            f"{self.clicked_point_topic}. For each point, the script records "
            "the clicked position, reads the robot's current heading, then "
            f"asks for the waypoint name. YAML is written to {self.waypoint_file}."
        )

    def _clicked_point_cb(self, msg):
        frame_id = msg.header.frame_id or self.map_frame
        event = {
            "frame_id": frame_id,
            "x": float(msg.point.x),
            "y": float(msg.point.y),
            "z": float(msg.point.z),
        }
        self.pending_points.put(event)
        self.node.get_logger().info(
            f"Recorded clicked position in {frame_id}: "
            f"x={event['x']:.3f}, y={event['y']:.3f}, z={event['z']:.3f}"
        )

    def _default_name(self):
        with self.waypoint_lock:
            return f"wp_{len(self.waypoints) + 1:02d}"

    def _unique_name(self, desired_name):
        with self.waypoint_lock:
            existing = {wp["name"] for wp in self.waypoints}

        if desired_name not in existing:
            return desired_name

        suffix = 2
        while f"{desired_name}_{suffix}" in existing:
            suffix += 1

        unique_name = f"{desired_name}_{suffix}"
        self.node.get_logger().warn(
            f"Waypoint name '{desired_name}' already exists, using '{unique_name}'."
        )
        return unique_name

    def _lookup_default_yaw(self, frame_id):
        import rclpy
        from rclpy.duration import Duration

        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            q = transform.transform.rotation
            return _yaw_from_quaternion(q.x, q.y, q.z, q.w), True
        except Exception as exc:
            self.node.get_logger().warn(
                f"Could not get default yaw from TF {frame_id} <- {self.base_frame}; "
                f"using 0 deg. Reason: {exc}"
            )
            return 0.0, False

    def _make_waypoint(self, event, name, yaw):
        quat = _quaternion_from_yaw(yaw)
        return {
            "name": name,
            "frame_id": event["frame_id"] or self.map_frame,
            "position": {
                "x": event["x"],
                "y": event["y"],
                "z": event["z"],
            },
            "yaw": yaw,
            "orientation": quat,
        }

    def _store_waypoint(self, waypoint, origin_label):
        with self.waypoint_lock:
            self.waypoints.append(waypoint)
        self._write_waypoints_file()
        self.node.get_logger().info(
            f"Saved waypoint '{waypoint['name']}' ({origin_label}) to {self.waypoint_file}"
        )

    def _prompt_loop(self):
        while not self.stop_event.is_set():
            try:
                event = self.pending_points.get(timeout=0.2)
            except queue.Empty:
                continue

            default_name = self._default_name()
            default_yaw, has_tf_yaw = self._lookup_default_yaw(event["frame_id"])
            default_yaw_deg = math.degrees(default_yaw)

            print(
                "\n[waypoint] step 1/3 position recorded: "
                f"frame={event['frame_id']} "
                f"x={event['x']:.3f} y={event['y']:.3f} z={event['z']:.3f}",
                flush=True,
            )
            if has_tf_yaw:
                print(
                    "[waypoint] step 2/3 current heading recorded: "
                    f"{default_yaw_deg:.1f} deg",
                    flush=True,
                )
            else:
                print(
                    "[waypoint] step 2/3 current heading unavailable, "
                    "using 0.0 deg",
                    flush=True,
                )

            try:
                raw_name = input(
                    f"[waypoint] step 3/3 waypoint name [{default_name}] "
                    "('skip' to ignore): "
                ).strip()
            except EOFError:
                raw_name = ""

            if self.stop_event.is_set():
                break
            if raw_name.lower() == "skip":
                self.node.get_logger().info("Skipped clicked point.")
                continue

            waypoint_name = self._unique_name(raw_name or default_name)
            waypoint = self._make_waypoint(event, waypoint_name, default_yaw)
            self._store_waypoint(waypoint, "interactive")

    def _write_waypoints_file(self):
        waypoint_path = os.path.expanduser(self.waypoint_file)
        os.makedirs(os.path.dirname(waypoint_path) or ".", exist_ok=True)

        tmp_path = waypoint_path + ".tmp"
        with self.waypoint_lock:
            waypoints_snapshot = list(self.waypoints)

        with open(tmp_path, "w", encoding="utf-8") as handle:
            if not waypoints_snapshot:
                handle.write("waypoints: []\n")
            else:
                handle.write("waypoints:\n")
                for wp in waypoints_snapshot:
                    handle.write(f"  - name: {_yaml_quote(wp['name'])}\n")
                    handle.write(f"    frame_id: {_yaml_quote(wp['frame_id'])}\n")
                    handle.write("    position:\n")
                    handle.write(f"      x: {wp['position']['x']:.6f}\n")
                    handle.write(f"      y: {wp['position']['y']:.6f}\n")
                    handle.write(f"      z: {wp['position']['z']:.6f}\n")
                    handle.write(f"    yaw: {wp['yaw']:.6f}\n")
                    handle.write("    orientation:\n")
                    handle.write(f"      x: {wp['orientation']['x']:.6f}\n")
                    handle.write(f"      y: {wp['orientation']['y']:.6f}\n")
                    handle.write(f"      z: {wp['orientation']['z']:.6f}\n")
                    handle.write(f"      w: {wp['orientation']['w']:.6f}\n")

        os.replace(tmp_path, waypoint_path)

    def _drain_pending_points(self):
        drained_count = 0
        while True:
            try:
                event = self.pending_points.get_nowait()
            except queue.Empty:
                break

            default_name = self._unique_name(self._default_name())
            default_yaw, _ = self._lookup_default_yaw(event["frame_id"])
            waypoint = self._make_waypoint(event, default_name, default_yaw)
            self._store_waypoint(waypoint, "shutdown-auto")
            drained_count += 1

        if drained_count:
            self.node.get_logger().warn(
                f"Auto-saved {drained_count} queued clicked points during shutdown."
            )

    def shutdown(self):
        self.stop_event.set()
        self._drain_pending_points()
        self._write_waypoints_file()
        self.node.get_logger().info(
            f"Waypoint YAML written to {os.path.expanduser(self.waypoint_file)}"
        )


def main(args=None):
    import rclpy
    from rclpy.node import Node

    if args is None:
        raw_args = sys.argv[1:]
    else:
        raw_args = list(args)

    config, ros_args = _parse_cli_args(raw_args)

    rclpy.init(args=ros_args)
    node = Node("save_map_and_waypoints")
    recorder = MapWaypointRecorder(node, config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
