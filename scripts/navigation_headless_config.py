#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2w_real.headless_config import (  # noqa: E402
    DEFAULT_HEADLESS_CONFIG_PATH,
    build_executor_settings,
    load_headless_config,
)


def _format_shell(runtime):
    lines = [
        f"HEADLESS_CONFIG_PATH={shlex.quote(runtime['config_path'])}",
        f"HEADLESS_PROFILE={shlex.quote(runtime['profile'])}",
        f"HEADLESS_WAYPOINT_FILE={shlex.quote(runtime['waypoint_file'])}",
        f"HEADLESS_SLAM_MAP_FILE={shlex.quote(runtime['slam_map_file'])}",
        f"HEADLESS_SERVER_URI={shlex.quote(runtime['server_uri'])}",
        f"HEADLESS_NETWORK_INTERFACE={shlex.quote(runtime['network_interface'])}",
        f"HEADLESS_SLAM_MODE={shlex.quote(runtime['slam_mode'])}",
        f"HEADLESS_NAV2_WAIT_TIMEOUT_SEC={runtime['nav2_wait_timeout_sec']}",
        f"HEADLESS_SERVER_TIMEOUT={runtime['server_timeout']}",
        f"HEADLESS_HEARTBEAT_INTERVAL={runtime['heartbeat_interval']}",
        f"HEADLESS_RECONNECT_DELAY={runtime['reconnect_delay']}",
        f"HEADLESS_NAV_EVENT_TIMEOUT={runtime['nav_event_timeout']}",
        f"HEADLESS_SHOW_LOGS={'1' if runtime['show_logs'] else '0'}",
        f"HEADLESS_RECORD_BAG={'1' if runtime['record_bag'] else '0'}",
        f"HEADLESS_USE_RVIZ={'1' if runtime['use_rviz'] else '0'}",
        f"HEADLESS_USE_ODOM_FUSION={'1' if runtime['use_odom_fusion'] else '0'}",
        f"HEADLESS_TERM_TIMEOUT_SEC={runtime['term_timeout_sec']}",
        f"HEADLESS_KILL_TIMEOUT_SEC={runtime['kill_timeout_sec']}",
        "HEADLESS_CLEANUP_MATCHERS=(",
    ]
    for matcher in runtime["process_matchers"]:
        lines.append(f"  {shlex.quote(matcher)}")
    lines.append(")")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_HEADLESS_CONFIG_PATH))
    parser.add_argument("--profile", default="")
    parser.add_argument("--format", choices=("shell",), default="shell")
    args = parser.parse_args()

    config = load_headless_config(args.config)
    runtime = build_executor_settings(config, args.profile or None)
    print(_format_shell(runtime))


if __name__ == "__main__":
    main()
