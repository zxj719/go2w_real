#!/usr/bin/env python3
"""
Record navigation waypoints from the robot's current localized pose.

Workflow:
  1. Start SLAM/localization so map -> base TF is available.
  2. Run this script.
  3. Press Enter to capture the robot's current pose in the map frame.
  4. The script auto-assigns a POI id and waypoint name.
  5. Press Ctrl+C when finished. The script overwrites waypoint YAML only.
"""

import math
import os
import sys
import threading


DEFAULT_WAYPOINT_FILE = "/home/unitree/ros_ws/src/go2w_waypoints.yaml"
DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base"
DEFAULT_POI_ID_START = 13


def _print_help():
    print(
        "Usage: ros2 run go2w_real save_map_and_waypoints.py [options] [--ros-args ...]\n"
        "\n"
        "Options:\n"
        "  --waypoint-file PATH       Waypoint YAML path, default: /home/unitree/ros_ws/src/go2w_waypoints.yaml\n"
        "  --map-frame FRAME          Map frame for saved waypoints, default: map\n"
        "  --base-frame FRAME         Robot base frame used for default yaw, default: base\n"
        "  -h, --help                 Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "waypoint_file": DEFAULT_WAYPOINT_FILE,
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


def _default_waypoint_name(index):
    return f"wp_{index:02d}"


def _default_waypoint_id(index, poi_start=DEFAULT_POI_ID_START):
    return f"POI_{poi_start + index - 1:03d}"


def _extract_yaml_prefix(existing_text):
    lines = existing_text.splitlines(keepends=True)
    for idx, line in enumerate(lines):
        if line.strip() == "waypoints:":
            return "".join(lines[:idx])
    return existing_text if existing_text.endswith("\n") or not existing_text else existing_text + "\n"


def _serialize_waypoints_yaml(waypoints, existing_text=""):
    parts = []
    prefix = _extract_yaml_prefix(existing_text)
    if prefix:
        parts.append(prefix)
        if not prefix.endswith("\n"):
            parts.append("\n")

    if not waypoints:
        parts.append("waypoints: []\n")
        return "".join(parts)

    parts.append("waypoints:\n")
    for wp in waypoints:
        parts.append(f"  - id: {_yaml_quote(wp['id'])}\n")
        parts.append(f"    name: {_yaml_quote(wp['name'])}\n")
        parts.append(f"    frame_id: {_yaml_quote(wp['frame_id'])}\n")
        parts.append("    position:\n")
        parts.append(f"      x: {wp['position']['x']:.6f}\n")
        parts.append(f"      y: {wp['position']['y']:.6f}\n")
        parts.append(f"      z: {wp['position']['z']:.6f}\n")
        parts.append(f"    yaw: {wp['yaw']:.6f}\n")
        parts.append("    orientation:\n")
        parts.append(f"      x: {wp['orientation']['x']:.6f}\n")
        parts.append(f"      y: {wp['orientation']['y']:.6f}\n")
        parts.append(f"      z: {wp['orientation']['z']:.6f}\n")
        parts.append(f"      w: {wp['orientation']['w']:.6f}\n")

    return "".join(parts)


class MapWaypointRecorder:
    def __init__(self, node, config):
        import tf2_ros

        self.node = node
        self.waypoint_file = config["waypoint_file"]
        self.map_frame = config["map_frame"]
        self.base_frame = config["base_frame"]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.node, spin_thread=False
        )

        self.waypoints = []
        self.waypoint_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.prompt_thread = threading.Thread(
            target=self._prompt_loop,
            name="waypoint_prompt",
            daemon=True,
        )
        self.prompt_thread.start()

        self.node.get_logger().info(
            "Waypoint recorder ready. Press Enter to capture the robot's "
            f"current pose from TF {self.map_frame} <- {self.base_frame}. "
            f"YAML is written to {self.waypoint_file}."
        )

    def _default_name(self):
        with self.waypoint_lock:
            return _default_waypoint_name(len(self.waypoints) + 1)

    def _default_id(self):
        with self.waypoint_lock:
            return _default_waypoint_id(len(self.waypoints) + 1)

    def _lookup_current_pose(self):
        import rclpy
        from rclpy.duration import Duration

        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return {
            "frame_id": self.map_frame,
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
            "yaw": yaw,
        }

    def _make_waypoint(self, pose, waypoint_id, name):
        yaw = pose["yaw"]
        quat = _quaternion_from_yaw(yaw)
        return {
            "id": waypoint_id,
            "name": name,
            "frame_id": pose["frame_id"] or self.map_frame,
            "position": {
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
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
                raw_text = input(
                    "[waypoint] Press Enter to capture current pose ('q' to quit): "
                )
            except EOFError:
                raw_text = "q"

            if self.stop_event.is_set():
                break

            command = raw_text.strip().lower()
            if command in ("q", "quit", "exit"):
                self.stop_event.set()
                break
            if command:
                self.node.get_logger().info(
                    "Ignored non-empty input. Press Enter to capture or q to quit."
                )
                continue

            try:
                pose = self._lookup_current_pose()
            except Exception as exc:
                self.node.get_logger().warn(
                    f"Could not get current pose from TF {self.map_frame} <- "
                    f"{self.base_frame}: {exc}"
                )
                continue

            waypoint_id = self._default_id()
            waypoint_name = self._default_name()
            waypoint = self._make_waypoint(pose, waypoint_id, waypoint_name)
            yaw_deg = math.degrees(waypoint["yaw"])
            self.node.get_logger().info(
                "Captured current pose: "
                f"id={waypoint_id}, name={waypoint_name}, "
                f"x={waypoint['position']['x']:.3f}, "
                f"y={waypoint['position']['y']:.3f}, "
                f"z={waypoint['position']['z']:.3f}, "
                f"yaw={yaw_deg:.1f} deg"
            )
            self._store_waypoint(waypoint, "interactive")

    def _write_waypoints_file(self):
        waypoint_path = os.path.expanduser(self.waypoint_file)
        os.makedirs(os.path.dirname(waypoint_path) or ".", exist_ok=True)

        tmp_path = waypoint_path + ".tmp"
        with self.waypoint_lock:
            waypoints_snapshot = list(self.waypoints)

        existing_text = ""
        if os.path.exists(waypoint_path):
            with open(waypoint_path, "r", encoding="utf-8") as handle:
                existing_text = handle.read()

        rendered_yaml = _serialize_waypoints_yaml(
            waypoints_snapshot,
            existing_text=existing_text,
        )
        with open(tmp_path, "w", encoding="utf-8") as handle:
            handle.write(rendered_yaml)

        os.replace(tmp_path, waypoint_path)

    def shutdown(self):
        self.stop_event.set()
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
