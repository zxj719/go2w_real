#!/usr/bin/env python3
"""
Sample CPU usage of the GO2W SLAM + Nav2 stack and group it by component.

This script uses /proc directly so it does not depend on psutil.
The reported CPU percentages follow the same convention as top/htop:
on multicore systems, a single busy process can exceed 100%.
"""

import os
import sys
import time


DEFAULT_INTERVAL = 1.0
DEFAULT_SAMPLES = 0

COMPONENT_PATTERNS = [
    ("go2w_bridge", ("go2w_bridge.py",)),
    ("pointcloud_relay", ("pointcloud_relay.py",)),
    ("pointcloud_to_laserscan", ("pointcloud_to_laserscan_node",)),
    ("laser_filters", ("scan_to_scan_filter_chain", "laser_scan_box_filter")),
    ("rf2o_laser_odometry", ("rf2o_laser_odometry_node",)),
    ("slam_toolbox", ("async_slam_toolbox_node",)),
    ("controller_server", ("nav2_controller/controller_server", "controller_server")),
    ("planner_server", ("nav2_planner/planner_server", "planner_server")),
    ("recoveries_server", ("nav2_recoveries/recoveries_server", "recoveries_server")),
    ("bt_navigator", ("nav2_bt_navigator/bt_navigator", "bt_navigator")),
    (
        "lifecycle_manager_navigation",
        ("lifecycle_manager_navigation", "nav2_lifecycle_manager/lifecycle_manager"),
    ),
    ("robot_state_publisher", ("robot_state_publisher",)),
    ("static_tf", ("static_transform_publisher",)),
    ("rviz2", ("rviz2",)),
    ("navigate_to_waypoint", ("navigate_to_waypoint.py",)),
]


def _print_help():
    print(
        "Usage: ros2 run go2w_real profile_nav_stack.py [options]\n"
        "\n"
        "Options:\n"
        "  --interval SEC        Sampling interval in seconds, default: 1.0\n"
        "  --samples N           Number of samples before exit, default: 0 (run until Ctrl+C)\n"
        "  --show-missing        Also list components that are not currently running\n"
        "  -h, --help            Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "interval": DEFAULT_INTERVAL,
        "samples": DEFAULT_SAMPLES,
        "show_missing": False,
    }

    i = 0
    while i < len(raw_args):
        arg = raw_args[i]

        if arg in ("-h", "--help"):
            _print_help()
            raise SystemExit(0)
        if arg == "--interval" and i + 1 < len(raw_args):
            config["interval"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--interval="):
            config["interval"] = float(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--samples" and i + 1 < len(raw_args):
            config["samples"] = int(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--samples="):
            config["samples"] = int(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--show-missing":
            config["show_missing"] = True
            i += 1
            continue

        raise SystemExit(f"Unknown argument: {arg}")

    if config["interval"] <= 0.0:
        raise SystemExit("--interval must be > 0")
    if config["samples"] < 0:
        raise SystemExit("--samples must be >= 0")

    return config


def _read_total_cpu_ticks():
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        cpu_fields = handle.readline().strip().split()[1:]
    return sum(int(value) for value in cpu_fields)


def _read_process_snapshot():
    snapshot = {}

    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue

        pid = int(entry)
        proc_dir = os.path.join("/proc", entry)

        try:
            with open(
                os.path.join(proc_dir, "cmdline"),
                "rb",
            ) as handle:
                raw_cmdline = handle.read()
            cmdline = raw_cmdline.replace(b"\x00", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
            if not cmdline:
                with open(
                    os.path.join(proc_dir, "comm"),
                    "r",
                    encoding="utf-8",
                ) as handle:
                    cmdline = handle.read().strip()

            with open(
                os.path.join(proc_dir, "stat"),
                "r",
                encoding="utf-8",
            ) as handle:
                stat_line = handle.read().strip()

            # /proc/<pid>/stat embeds comm in parentheses, so slice after the last ')'.
            fields = stat_line[stat_line.rfind(")") + 2 :].split()
            utime = int(fields[11])
            stime = int(fields[12])

            snapshot[pid] = {
                "cpu_ticks": utime + stime,
                "cmdline": cmdline,
            }
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError):
            continue

    return snapshot


def _classify_process(cmdline):
    for component, patterns in COMPONENT_PATTERNS:
        if any(pattern in cmdline for pattern in patterns):
            return component
    return None


def _format_rows(rows):
    if not rows:
        return ["  (no matching processes found)"]

    header = f"{'Component':<30} {'CPU%':>8} {'PIDs':>6}"
    lines = [header, "-" * len(header)]
    for row in rows:
        pid_count = len(row["pids"])
        lines.append(
            f"{row['component']:<30} {row['cpu_percent']:>7.1f} {pid_count:>6}"
        )
    return lines


def _compute_component_rows(prev_snapshot, curr_snapshot, total_delta, cpu_count):
    grouped = {}

    for pid, curr in curr_snapshot.items():
        prev = prev_snapshot.get(pid)
        if prev is None:
            continue

        delta_ticks = curr["cpu_ticks"] - prev["cpu_ticks"]
        if delta_ticks < 0:
            continue

        component = _classify_process(curr["cmdline"])
        if component is None:
            continue

        if component not in grouped:
            grouped[component] = {"cpu_ticks": 0, "pids": []}

        grouped[component]["cpu_ticks"] += delta_ticks
        grouped[component]["pids"].append(pid)

    rows = []
    for component, _patterns in COMPONENT_PATTERNS:
        data = grouped.get(component)
        if data is None:
            continue

        cpu_percent = 100.0 * cpu_count * float(data["cpu_ticks"]) / float(total_delta)
        rows.append(
            {
                "component": component,
                "cpu_percent": cpu_percent,
                "pids": sorted(set(data["pids"])),
            }
        )

    rows.sort(key=lambda item: item["cpu_percent"], reverse=True)
    return rows


def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)
    config = _parse_cli_args(raw_args)

    cpu_count = os.cpu_count() or 1
    interval = config["interval"]
    show_missing = config["show_missing"]
    samples_limit = config["samples"]

    print(
        "GO2W Nav Stack CPU profiler started.\n"
        f"- interval: {interval:.2f} s\n"
        f"- cpu_count: {cpu_count}\n"
        "- stop with Ctrl+C\n",
        flush=True,
    )

    prev_total = _read_total_cpu_ticks()
    prev_snapshot = _read_process_snapshot()
    samples_done = 0

    try:
        while True:
            time.sleep(interval)
            curr_total = _read_total_cpu_ticks()
            curr_snapshot = _read_process_snapshot()

            total_delta = curr_total - prev_total
            if total_delta <= 0:
                prev_total = curr_total
                prev_snapshot = curr_snapshot
                continue

            rows = _compute_component_rows(
                prev_snapshot,
                curr_snapshot,
                total_delta,
                cpu_count,
            )
            stack_total = sum(row["cpu_percent"] for row in rows)

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            print(
                f"\n[{timestamp}] sample {samples_done + 1} | "
                f"stack_total={stack_total:.1f}%",
                flush=True,
            )
            for line in _format_rows(rows):
                print(line, flush=True)

            if show_missing:
                active_components = {row["component"] for row in rows}
                missing = [
                    component
                    for component, _patterns in COMPONENT_PATTERNS
                    if component not in active_components
                ]
                if missing:
                    print("Missing: " + ", ".join(missing), flush=True)

            prev_total = curr_total
            prev_snapshot = curr_snapshot
            samples_done += 1

            if samples_limit and samples_done >= samples_limit:
                break
    except KeyboardInterrupt:
        print("\nProfiler stopped.", flush=True)


if __name__ == "__main__":
    main()
