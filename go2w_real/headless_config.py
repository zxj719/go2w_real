from __future__ import annotations

from pathlib import Path

import yaml


def _resolve_default_headless_config_path():
    module_path = Path(__file__).resolve()
    source_candidate = module_path.parents[1] / "config" / "navigation_headless.yaml"
    if source_candidate.is_file():
        return source_candidate

    for parent in module_path.parents:
        share_candidate = (
            parent / "share" / "go2w_real" / "config" / "navigation_headless.yaml"
        )
        if share_candidate.is_file():
            return share_candidate

    return source_candidate


DEFAULT_HEADLESS_CONFIG_PATH = _resolve_default_headless_config_path()


class HeadlessConfigError(ValueError):
    pass


def load_headless_config(config_path=DEFAULT_HEADLESS_CONFIG_PATH):
    config_path = Path(config_path).expanduser()
    if not config_path.is_file():
        raise HeadlessConfigError(f"headless config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise HeadlessConfigError(
            f"invalid headless config root in {config_path}"
        )

    data["_config_path"] = str(config_path)
    return data


def resolve_headless_profile(config, profile_name=None):
    profiles = config.get("profiles") or {}
    if not isinstance(profiles, dict) or not profiles:
        raise HeadlessConfigError("headless config must define profiles")

    selected_name = profile_name or config.get("default_profile")
    selected = profiles.get(selected_name)
    if not isinstance(selected, dict):
        raise HeadlessConfigError(f"unknown headless profile: {selected_name}")

    waypoint_file = str(selected.get("waypoint_file", "")).strip()
    slam_map_file = str(selected.get("slam_map_file", "")).strip()
    if not waypoint_file or not slam_map_file:
        raise HeadlessConfigError(
            f"headless profile '{selected_name}' is missing waypoint_file or slam_map_file"
        )

    return {
        "name": selected_name,
        "waypoint_file": waypoint_file,
        "slam_map_file": slam_map_file,
    }


def build_nav2_param_rewrites(config):
    nav2_cfg = config.get("nav2") or {}
    return {
        "xy_goal_tolerance": str(nav2_cfg.get("xy_goal_tolerance", 0.5)),
        "yaw_goal_tolerance": str(nav2_cfg.get("yaw_goal_tolerance", 3.14)),
    }


def build_localization_param_rewrites(config):
    slam_cfg = config.get("slam_localization") or {}
    return {
        "do_loop_closing": str(slam_cfg.get("do_loop_closing", False)).lower(),
        "link_match_minimum_response_fine": str(
            slam_cfg.get("link_match_minimum_response_fine", 0.2)
        ),
        "loop_match_minimum_response_coarse": str(
            slam_cfg.get("loop_match_minimum_response_coarse", 0.5)
        ),
        "loop_match_minimum_response_fine": str(
            slam_cfg.get("loop_match_minimum_response_fine", 0.6)
        ),
    }


def build_executor_settings(config, profile_name=None):
    profile = resolve_headless_profile(config, profile_name)
    executor_cfg = config.get("executor") or {}
    launch_cfg = config.get("launch") or {}
    cleanup_cfg = config.get("cleanup") or {}

    process_matchers = cleanup_cfg.get("process_matchers", [])
    if not isinstance(process_matchers, list):
        raise HeadlessConfigError("cleanup.process_matchers must be a list")

    return {
        "profile": profile["name"],
        "waypoint_file": profile["waypoint_file"],
        "slam_map_file": profile["slam_map_file"],
        "server_uri": str(executor_cfg.get("server_uri", "")).strip(),
        "server_timeout": float(executor_cfg.get("server_timeout", 10.0)),
        "heartbeat_interval": float(executor_cfg.get("heartbeat_interval", 30.0)),
        "reconnect_delay": float(executor_cfg.get("reconnect_delay", 3.0)),
        "nav_event_timeout": float(executor_cfg.get("nav_event_timeout", 15.0)),
        "show_logs": bool(executor_cfg.get("show_logs", True)),
        "record_bag": bool(executor_cfg.get("record_bag", True)),
        "network_interface": str(
            launch_cfg.get("network_interface", "eth0")
        ).strip(),
        "use_rviz": bool(launch_cfg.get("use_rviz", False)),
        "slam_mode": str(launch_cfg.get("slam_mode", "localization")).strip(),
        "use_odom_fusion": bool(launch_cfg.get("use_odom_fusion", False)),
        "nav2_wait_timeout_sec": int(launch_cfg.get("nav2_wait_timeout_sec", 120)),
        "process_matchers": [str(matcher).strip() for matcher in process_matchers],
        "term_timeout_sec": int(cleanup_cfg.get("term_timeout_sec", 5)),
        "kill_timeout_sec": int(cleanup_cfg.get("kill_timeout_sec", 2)),
        "config_path": str(config.get("_config_path", DEFAULT_HEADLESS_CONFIG_PATH)),
    }
