#!/usr/bin/env python3
"""
Sample CPU usage of the GO2W SLAM + Nav2 stack and persist a full per-node
history until the profiler exits.

This script uses /proc directly so it does not depend on psutil.
The reported CPU percentages follow the same convention as top/htop:
on multicore systems, a single busy process can exceed 100%.
"""

import csv
import os
import sys
import time


DEFAULT_INTERVAL = 1.0
DEFAULT_SAMPLES = 0
DEFAULT_OUTPUT_ROOT = "profile_nav_stack_logs"

MONITORED_COMPONENTS = [
    {
        "name": "xt16_driver",
        "cmd_patterns": ("xt16_driver",),
        "node_names": (),
    },
    {
        "name": "go2w_bridge",
        "cmd_patterns": ("go2w_bridge.py",),
        "node_names": ("go2w_bridge",),
    },
    {
        "name": "pointcloud_relay",
        "cmd_patterns": ("pointcloud_relay.py",),
        "node_names": ("pointcloud_relay",),
    },
    {
        "name": "pointcloud_to_laserscan",
        "cmd_patterns": ("pointcloud_to_laserscan_node",),
        "node_names": ("pointcloud_to_laserscan",),
    },
    {
        "name": "scan_to_scan_filter_chain",
        "cmd_patterns": ("scan_to_scan_filter_chain", "laser_scan_box_filter"),
        "node_names": ("scan_to_scan_filter_chain", "laser_scan_box_filter"),
    },
    {
        "name": "rf2o_laser_odometry",
        "cmd_patterns": ("rf2o_laser_odometry_node",),
        "node_names": ("rf2o_laser_odometry",),
    },
    {
        "name": "wait_for_odom_to_lidar_tf",
        "cmd_patterns": (),
        "node_names": ("wait_for_odom_to_lidar_tf",),
    },
    {
        "name": "wait_for_map_to_lidar_tf",
        "cmd_patterns": (),
        "node_names": ("wait_for_map_to_lidar_tf",),
    },
    {
        "name": "slam_toolbox",
        "cmd_patterns": (
            "localization_slam_toolbox_node",
            "async_slam_toolbox_node",
        ),
        "node_names": ("slam_toolbox",),
    },
    {
        "name": "controller_server",
        "cmd_patterns": ("nav2_controller/controller_server", "controller_server"),
        "node_names": ("controller_server",),
    },
    {
        "name": "planner_server",
        "cmd_patterns": ("nav2_planner/planner_server", "planner_server"),
        "node_names": ("planner_server",),
    },
    {
        "name": "recoveries_server",
        "cmd_patterns": (
            "nav2_recoveries/recoveries_server",
            "recoveries_server",
        ),
        "node_names": ("recoveries_server",),
    },
    {
        "name": "bt_navigator",
        "cmd_patterns": ("nav2_bt_navigator/bt_navigator", "bt_navigator"),
        "node_names": ("bt_navigator",),
    },
    {
        "name": "lifecycle_manager_navigation",
        "cmd_patterns": (
            "lifecycle_manager_navigation",
            "nav2_lifecycle_manager/lifecycle_manager",
        ),
        "node_names": ("lifecycle_manager_navigation",),
    },
    {
        "name": "robot_state_publisher",
        "cmd_patterns": ("robot_state_publisher",),
        "node_names": ("robot_state_publisher",),
    },
    {
        "name": "base_to_base_footprint",
        "cmd_patterns": (),
        "node_names": ("base_to_base_footprint",),
    },
    {
        "name": "lidar_to_rslidar",
        "cmd_patterns": (),
        "node_names": ("lidar_to_rslidar",),
    },
    {
        "name": "rviz2",
        "cmd_patterns": ("rviz2",),
        "node_names": ("rviz2",),
    },
    {
        "name": "navigate_to_waypoint",
        "cmd_patterns": ("navigate_to_waypoint.py",),
        "node_names": ("navigate_to_waypoint",),
    },
    {
        "name": "navigation_executor",
        "cmd_patterns": ("navigation_executor.py",),
        "node_names": ("navigation_executor",),
    },
]

COMPONENT_NAMES = [component["name"] for component in MONITORED_COMPONENTS]


def _print_help():
    print(
        "Usage: ros2 run go2w_real profile_nav_stack.py [options]\n"
        "\n"
        "Options:\n"
        "  --interval SEC        Sampling interval in seconds, default: 1.0\n"
        "  --samples N           Number of samples before exit, default: 0 (run until Ctrl+C)\n"
        "  --output-dir DIR      Directory for samples.csv / summary.csv / summary.md\n"
        "                        default: ./profile_nav_stack_logs/<timestamp>\n"
        "  --show-missing        Also show nodes with 0 CPU in the realtime table\n"
        "  -h, --help            Show this help message\n"
    )


def _parse_cli_args(raw_args):
    config = {
        "interval": DEFAULT_INTERVAL,
        "samples": DEFAULT_SAMPLES,
        "output_dir": None,
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
        if arg == "--output-dir" and i + 1 < len(raw_args):
            config["output_dir"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        if arg.startswith("--output-dir="):
            config["output_dir"] = os.path.expanduser(arg.split("=", 1)[1])
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


def _extract_ros_node_name(tokens):
    for token in tokens:
        if "__node:=" in token:
            value = token.split("__node:=", 1)[1].strip()
            return value.strip("/").split("/")[-1]
        if "__name:=" in token:
            value = token.split("__name:=", 1)[1].strip()
            return value.strip("/").split("/")[-1]
    return None


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

            tokens = [
                token.decode("utf-8", errors="replace")
                for token in raw_cmdline.split(b"\x00")
                if token
            ]

            if tokens:
                cmdline = " ".join(tokens).strip()
            else:
                with open(
                    os.path.join(proc_dir, "comm"),
                    "r",
                    encoding="utf-8",
                ) as handle:
                    cmdline = handle.read().strip()
                tokens = [cmdline] if cmdline else []

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
                "ros_node_name": _extract_ros_node_name(tokens),
            }
        except (
            FileNotFoundError,
            ProcessLookupError,
            PermissionError,
            IndexError,
            ValueError,
        ):
            continue

    return snapshot


def _classify_process(process_info):
    ros_node_name = process_info.get("ros_node_name")
    cmdline = process_info["cmdline"]

    if ros_node_name:
        for component in MONITORED_COMPONENTS:
            if ros_node_name in component["node_names"]:
                return component["name"]

    for component in MONITORED_COMPONENTS:
        if any(pattern in cmdline for pattern in component["cmd_patterns"]):
            return component["name"]

    return None


def _format_rows(rows, show_missing=False):
    visible_rows = [
        row
        for row in rows
        if show_missing or row["present"] or row["cpu_percent"] > 0.0
    ]

    if not visible_rows:
        return ["  (no matching processes found)"]

    visible_rows.sort(key=lambda item: item["cpu_percent"], reverse=True)

    header = f"{'Component':<30} {'CPU%':>8} {'PIDs':>6}"
    lines = [header, "-" * len(header)]
    for row in visible_rows:
        pid_count = len(row["pids"])
        lines.append(
            f"{row['component']:<30} {row['cpu_percent']:>7.1f} {pid_count:>6}"
        )
    return lines


def _compute_component_rows(prev_snapshot, curr_snapshot, total_delta, cpu_count):
    grouped = {
        component: {
            "cpu_ticks": 0,
            "pids": [],
        }
        for component in COMPONENT_NAMES
    }

    for pid, curr in curr_snapshot.items():
        component = _classify_process(curr)
        if component is None:
            continue

        grouped[component]["pids"].append(pid)

        prev = prev_snapshot.get(pid)
        if prev is None:
            continue

        delta_ticks = curr["cpu_ticks"] - prev["cpu_ticks"]
        if delta_ticks < 0:
            continue

        grouped[component]["cpu_ticks"] += delta_ticks

    rows = []
    for component in COMPONENT_NAMES:
        data = grouped[component]
        cpu_percent = 0.0
        if total_delta > 0:
            cpu_percent = (
                100.0
                * cpu_count
                * float(data["cpu_ticks"])
                / float(total_delta)
            )

        rows.append(
            {
                "component": component,
                "cpu_percent": cpu_percent,
                "pids": sorted(set(data["pids"])),
                "present": bool(data["pids"]),
            }
        )

    return rows


def _default_output_dir():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.abspath(
        os.path.join(os.getcwd(), DEFAULT_OUTPUT_ROOT, timestamp)
    )


def _prepare_output_dir(path_from_cli):
    if path_from_cli:
        output_dir = os.path.abspath(path_from_cli)
    else:
        output_dir = _default_output_dir()

    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def _init_stats():
    return {
        component: {
            "samples": 0,
            "sum_cpu": 0.0,
            "active_samples": 0,
            "active_sum_cpu": 0.0,
            "min_cpu": None,
            "max_cpu": 0.0,
        }
        for component in COMPONENT_NAMES
    }


def _update_stats(component_stats, cpu_percent, present):
    component_stats["samples"] += 1
    component_stats["sum_cpu"] += cpu_percent

    if component_stats["min_cpu"] is None:
        component_stats["min_cpu"] = cpu_percent
    else:
        component_stats["min_cpu"] = min(component_stats["min_cpu"], cpu_percent)

    component_stats["max_cpu"] = max(component_stats["max_cpu"], cpu_percent)

    if present:
        component_stats["active_samples"] += 1
        component_stats["active_sum_cpu"] += cpu_percent


def _build_summary_rows(stats):
    rows = []

    for component in COMPONENT_NAMES:
        component_stats = stats[component]
        samples = component_stats["samples"]
        active_samples = component_stats["active_samples"]

        avg_cpu = component_stats["sum_cpu"] / samples if samples else 0.0
        active_avg_cpu = (
            component_stats["active_sum_cpu"] / active_samples
            if active_samples
            else 0.0
        )

        rows.append(
            {
                "component": component,
                "avg_cpu_percent": avg_cpu,
                "avg_cpu_when_running_percent": active_avg_cpu,
                "min_cpu_percent": component_stats["min_cpu"]
                if component_stats["min_cpu"] is not None
                else 0.0,
                "max_cpu_percent": component_stats["max_cpu"],
                "samples": samples,
                "active_samples": active_samples,
            }
        )

    rows.sort(key=lambda item: item["avg_cpu_percent"], reverse=True)
    return rows


def _write_summary_csv(path, summary_rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "component",
                "avg_cpu_percent",
                "avg_cpu_when_running_percent",
                "min_cpu_percent",
                "max_cpu_percent",
                "samples",
                "active_samples",
            ],
        )
        writer.writeheader()
        for row in summary_rows:
            writer.writerow(
                {
                    "component": row["component"],
                    "avg_cpu_percent": f"{row['avg_cpu_percent']:.3f}",
                    "avg_cpu_when_running_percent": (
                        f"{row['avg_cpu_when_running_percent']:.3f}"
                    ),
                    "min_cpu_percent": f"{row['min_cpu_percent']:.3f}",
                    "max_cpu_percent": f"{row['max_cpu_percent']:.3f}",
                    "samples": row["samples"],
                    "active_samples": row["active_samples"],
                }
            )


def _write_summary_markdown(
    path,
    summary_rows,
    sample_count,
    interval,
    cpu_count,
    started_at,
    stopped_at,
    output_dir,
):
    monitored_seconds = sample_count * interval
    lines = [
        "# GO2W Nav Stack CPU Profiling Report",
        "",
        "## Run Info",
        "",
        f"- Started at: {started_at}",
        f"- Stopped at: {stopped_at}",
        f"- Sampling interval: {interval:.2f} s",
        f"- Samples collected: {sample_count}",
        f"- Approx. monitored time: {monitored_seconds:.2f} s",
        f"- CPU count: {cpu_count}",
        f"- Output directory: `{output_dir}`",
        "",
        "## Average CPU Usage Per Node",
        "",
        "| Node | Avg CPU% (whole run) | Avg CPU% (when running) | Min CPU% | Max CPU% | Seen samples |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]

    for row in summary_rows:
        lines.append(
            "| "
            f"{row['component']} | "
            f"{row['avg_cpu_percent']:.3f} | "
            f"{row['avg_cpu_when_running_percent']:.3f} | "
            f"{row['min_cpu_percent']:.3f} | "
            f"{row['max_cpu_percent']:.3f} | "
            f"{row['active_samples']}/{row['samples']} |"
        )

    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `Avg CPU% (whole run)` treats samples before a node starts, after it exits, or while it is absent as `0.0`.",
            "- `Avg CPU% (when running)` only uses samples where the process was detected in `/proc`.",
            "- The detailed per-sample history is stored in `samples.csv` in the same output directory.",
            "",
        ]
    )

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def _print_final_summary(summary_rows):
    print("\nAverage CPU usage across the full monitoring window:", flush=True)
    header = f"{'Component':<30} {'Avg%':>8} {'RunAvg%':>10} {'Max%':>8} {'Seen':>10}"
    print(header, flush=True)
    print("-" * len(header), flush=True)

    for row in summary_rows:
        seen = f"{row['active_samples']}/{row['samples']}"
        print(
            f"{row['component']:<30} "
            f"{row['avg_cpu_percent']:>7.1f} "
            f"{row['avg_cpu_when_running_percent']:>9.1f} "
            f"{row['max_cpu_percent']:>7.1f} "
            f"{seen:>10}",
            flush=True,
        )


def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)
    config = _parse_cli_args(raw_args)

    cpu_count = os.cpu_count() or 1
    interval = config["interval"]
    show_missing = config["show_missing"]
    samples_limit = config["samples"]
    output_dir = _prepare_output_dir(config["output_dir"])
    samples_path = os.path.join(output_dir, "samples.csv")
    summary_csv_path = os.path.join(output_dir, "summary.csv")
    summary_md_path = os.path.join(output_dir, "summary.md")

    started_at = time.strftime("%Y-%m-%d %H:%M:%S")
    print(
        "GO2W Nav Stack CPU profiler started.\n"
        f"- interval: {interval:.2f} s\n"
        f"- cpu_count: {cpu_count}\n"
        f"- output_dir: {output_dir}\n"
        "- monitoring xt16_driver and the GO2W SLAM/Nav2 nodes\n"
        "- stop with Ctrl+C\n",
        flush=True,
    )

    prev_total = _read_total_cpu_ticks()
    prev_snapshot = _read_process_snapshot()
    samples_done = 0
    stats = _init_stats()
    start_monotonic = time.monotonic()

    with open(samples_path, "w", newline="", encoding="utf-8") as samples_handle:
        writer = csv.DictWriter(
            samples_handle,
            fieldnames=[
                "sample_index",
                "timestamp",
                "elapsed_sec",
                "stack_total_cpu_percent",
            ]
            + COMPONENT_NAMES,
        )
        writer.writeheader()
        samples_handle.flush()

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
                row_map = {row["component"]: row for row in rows}
                stack_total = sum(row["cpu_percent"] for row in rows)

                sample_timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                elapsed_sec = time.monotonic() - start_monotonic
                sample_index = samples_done + 1

                csv_row = {
                    "sample_index": sample_index,
                    "timestamp": sample_timestamp,
                    "elapsed_sec": f"{elapsed_sec:.3f}",
                    "stack_total_cpu_percent": f"{stack_total:.3f}",
                }

                for component in COMPONENT_NAMES:
                    row = row_map[component]
                    _update_stats(stats[component], row["cpu_percent"], row["present"])
                    csv_row[component] = f"{row['cpu_percent']:.3f}"

                writer.writerow(csv_row)
                samples_handle.flush()

                print(
                    f"\n[{sample_timestamp}] sample {sample_index} | "
                    f"stack_total={stack_total:.1f}%",
                    flush=True,
                )
                for line in _format_rows(rows, show_missing=show_missing):
                    print(line, flush=True)

                prev_total = curr_total
                prev_snapshot = curr_snapshot
                samples_done += 1

                if samples_limit and samples_done >= samples_limit:
                    break
        except KeyboardInterrupt:
            print("\nProfiler stopped.", flush=True)

    stopped_at = time.strftime("%Y-%m-%d %H:%M:%S")
    summary_rows = _build_summary_rows(stats)
    _write_summary_csv(summary_csv_path, summary_rows)
    _write_summary_markdown(
        summary_md_path,
        summary_rows,
        samples_done,
        interval,
        cpu_count,
        started_at,
        stopped_at,
        output_dir,
    )
    _print_final_summary(summary_rows)

    print(
        "\nReports written to:\n"
        f"- {samples_path}\n"
        f"- {summary_csv_path}\n"
        f"- {summary_md_path}",
        flush=True,
    )


if __name__ == "__main__":
    main()
