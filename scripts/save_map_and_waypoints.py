#!/usr/bin/env python3
"""
Record named navigation waypoints from RViz Publish Point and save the map on exit.

Workflow:
  1. Start SLAM and drive the robot to build the map.
  2. Run this script.
  3. In RViz, use "Publish Point" to click waypoint positions.
  4. For each clicked point, enter a waypoint name in the terminal.
  5. Optionally enter a yaw angle in degrees; pressing Enter uses the robot's
     current heading in the clicked point's frame.
  6. Press Ctrl+C when finished. The script writes the waypoint YAML and runs
     nav2_map_server's map_saver_cli.
"""

import math
import os
import queue
import shutil
import subprocess
import sys
import threading


DEFAULT_MAP_PREFIX = os.path.expanduser("~/maps/go2w_map")
DEFAULT_CLICKED_POINT_TOPIC = "/clicked_point"
DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base"


def _print_help():
    print(
        "Usage: ros2 run go2w_real save_map_and_waypoints.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        "  --map-prefix PATH          Map save prefix, default: ~/maps/go2w_map\n"
        "  --waypoint-file PATH       Waypoint YAML path, default: <map-prefix>_waypoints.yaml\n"
        "  --clicked-point-topic TOPIC\n"
        "                             RViz Publish Point topic, default: /clicked_point\n"
        "  --map-frame FRAME          Map frame for saved waypoints, default: map\n"
        "  --base-frame FRAME         Robot base frame used for default yaw, default: base\n"
        "  --no-save-map              Only write waypoint YAML, skip map_saver_cli\n"
        "  -h, --help                 Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "map_prefix": DEFAULT_MAP_PREFIX,
        "waypoint_file": None,
        "clicked_point_topic": DEFAULT_CLICKED_POINT_TOPIC,
        "map_frame": DEFAULT_MAP_FRAME,
        "base_frame": DEFAULT_BASE_FRAME,
        "save_map_on_shutdown": True,
    }
    ros_args = []

    i = 0
    while i < len(raw_args):
        arg = raw_args[i]

        if arg in ("-h", "--help"):
            _print_help()
            raise SystemExit(0)
        if arg == "--map-prefix" and i + 1 < len(raw_args):
            config["map_prefix"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--map-prefix="):
            config["map_prefix"] = os.path.expanduser(arg.split("=", 1)[1])
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
        if arg == "--no-save-map":
            config["save_map_on_shutdown"] = False
            i += 1
            continue

        ros_args.append(arg)
        i += 1

    if config["waypoint_file"] is None:
        config["waypoint_file"] = config["map_prefix"] + "_waypoints.yaml"

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
        self.map_prefix = config["map_prefix"]
        self.waypoint_file = config["waypoint_file"]
        self.clicked_point_topic = config["clicked_point_topic"]
        self.map_frame = config["map_frame"]
        self.base_frame = config["base_frame"]
        self.save_map_on_shutdown = config["save_map_on_shutdown"]

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
            f"{self.clicked_point_topic}, then name each point in this terminal. "
            "Press Ctrl+C to save the map and exit."
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
            f"Queued clicked point in {frame_id}: "
            f"({event['x']:.3f}, {event['y']:.3f}, {event['z']:.3f})"
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
                "\n[waypoint] clicked point: "
                f"frame={event['frame_id']} "
                f"x={event['x']:.3f} y={event['y']:.3f} z={event['z']:.3f}",
                flush=True,
            )
            if has_tf_yaw:
                print(
                    f"[waypoint] default yaw from current robot heading: {default_yaw_deg:.1f} deg",
                    flush=True,
                )

            try:
                raw_name = input(
                    f"[waypoint] name [{default_name}] ('skip' to ignore): "
                ).strip()
            except EOFError:
                raw_name = ""

            if self.stop_event.is_set():
                break
            if raw_name.lower() == "skip":
                self.node.get_logger().info("Skipped clicked point.")
                continue

            waypoint_name = self._unique_name(raw_name or default_name)

            try:
                raw_yaw_deg = input(
                    f"[waypoint] yaw_deg [{default_yaw_deg:.1f}]: "
                ).strip()
            except EOFError:
                raw_yaw_deg = ""

            if self.stop_event.is_set():
                break

            if raw_yaw_deg.lower() == "skip":
                self.node.get_logger().info("Skipped clicked point.")
                continue

            if raw_yaw_deg:
                try:
                    yaw = math.radians(float(raw_yaw_deg))
                except ValueError:
                    self.node.get_logger().warn(
                        f"Invalid yaw '{raw_yaw_deg}', using default {default_yaw_deg:.1f} deg."
                    )
                    yaw = default_yaw
            else:
                yaw = default_yaw

            waypoint = self._make_waypoint(event, waypoint_name, yaw)
            self._store_waypoint(waypoint, "interactive")

    def _write_waypoints_file(self):
        waypoint_path = os.path.expanduser(self.waypoint_file)
        map_prefix = os.path.expanduser(self.map_prefix)
        os.makedirs(os.path.dirname(waypoint_path) or ".", exist_ok=True)

        tmp_path = waypoint_path + ".tmp"
        with self.waypoint_lock:
            waypoints_snapshot = list(self.waypoints)

        with open(tmp_path, "w", encoding="utf-8") as handle:
            handle.write("map:\n")
            handle.write(f"  prefix: {_yaml_quote(map_prefix)}\n")
            handle.write(f"  yaml: {_yaml_quote(map_prefix + '.yaml')}\n")
            handle.write(f"  image: {_yaml_quote(map_prefix + '.pgm')}\n")
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

    def _save_map(self):
        if not self.save_map_on_shutdown:
            self.node.get_logger().info("Skipping map save on shutdown.")
            return

        os.makedirs(os.path.dirname(self.map_prefix) or ".", exist_ok=True)
        ros2_bin = shutil.which("ros2")
        if ros2_bin is None:
            self.node.get_logger().error(
                "Could not find 'ros2' in PATH, skipping map save."
            )
            return

        cmd = [ros2_bin, "run", "nav2_map_server", "map_saver_cli", "-f", self.map_prefix]
        self.node.get_logger().info(
            f"Saving map to prefix {self.map_prefix} with map_saver_cli..."
        )

        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=120,
                env=os.environ.copy(),
            )
        except Exception as exc:
            self.node.get_logger().error(f"Map save failed: {exc}")
            return

        if result.returncode != 0:
            self.node.get_logger().error(
                f"map_saver_cli failed with code {result.returncode}:\n{result.stdout}"
            )
            return

        self.node.get_logger().info(
            f"Map saved successfully:\n{result.stdout.strip()}"
        )

    def shutdown(self):
        self.stop_event.set()
        self._drain_pending_points()
        self._write_waypoints_file()
        self._save_map()


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
