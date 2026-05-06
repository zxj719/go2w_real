import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_MANAGER_PATH = PACKAGE_ROOT / "operator_console/launch_manager.py"
PROFILES_PATH = PACKAGE_ROOT / "operator_console/profiles.yaml"
WEB_INDEX_PATH = PACKAGE_ROOT / "operator_console/web/index.html"
START_HEADLESS_SCRIPT = PACKAGE_ROOT / "scripts/start_navigation_headless.sh"


def _load_launch_manager_module():
    spec = importlib.util.spec_from_file_location(
        "go2w_operator_console_launch_manager",
        LAUNCH_MANAGER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


launch_manager = _load_launch_manager_module()


def test_profiles_expose_headless_debug_bag_and_visualization_config():
    loaded = launch_manager.load_profiles(PROFILES_PATH)

    assert "navigation_headless_ws" in loaded["profiles"]
    assert "start_navigation_headless.sh" in loaded["profiles"]["navigation_headless_ws"]["command"]
    assert "navigation_headless_rviz" in loaded["profiles"]
    assert (
        loaded["profiles"]["navigation_headless_rviz"]["env"]["GO2W_HEADLESS_USE_RVIZ"]
        == "1"
    )

    debug_cfg = loaded["debug_navigation"]
    assert "navigate_to_waypoint_with_debug_bag.sh" in debug_cfg["command"]
    assert debug_cfg["default_profile"] == "zt_0"
    assert debug_cfg["allowed_profiles"] == ["zt_0", "14_1"]
    assert debug_cfg["waypoint_files"]["zt_0"].endswith(
        "/config/go2w_waypoints.yaml"
    )
    assert debug_cfg["waypoint_files"]["14_1"].endswith(
        "/config/go2w_waypoints.yaml"
    )

    visualization_cfg = loaded["visualization"]
    assert visualization_cfg["foxglove_port"] == 8765
    assert "{{host}}" in visualization_cfg["rviz_url"]


def test_debug_bag_args_include_only_whitelisted_tuning_parameters():
    args = launch_manager.build_debug_bag_args(
        "/tmp/navigate_to_waypoint_with_debug_bag.sh",
        allowed_profiles=["zt_0", "14_1"],
        profile_waypoint_files={
            "14_1": "/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml",
        },
        request={
            "profile": "14_1",
            "target_id": "POI_019",
            "no_bringup": True,
            "use_waypoint_yaw": True,
            "near_goal_distance": 0.35,
            "success_check_distance": 0.8,
            "abort_replan_retries": 2,
            "nav_ready_timeout": 90,
            "tf_ready_timeout": 12.5,
            "scan_ready_timeout": 8,
            "scan_ready_min_hz": 6.5,
            "xt16_ready_timeout": 11,
            "xt16_ready_min_hz": 9.0,
            "xt16_ready_restarts": 3,
            "ignored_shell": "; rm -rf /",
        },
    )

    assert args == [
        "/tmp/navigate_to_waypoint_with_debug_bag.sh",
        "--profile",
        "14_1",
        "--waypoint-file",
        "/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml",
        "--no-bringup",
        "--nav-ready-timeout",
        "90",
        "--tf-ready-timeout",
        "12.5",
        "--scan-ready-timeout",
        "8",
        "--scan-ready-min-hz",
        "6.5",
        "--xt16-ready-timeout",
        "11",
        "--xt16-ready-min-hz",
        "9.0",
        "--xt16-ready-restarts",
        "3",
        "--",
        "--waypoint",
        "POI_019",
        "--near-goal-distance",
        "0.35",
        "--success-check-distance",
        "0.8",
        "--abort-replan-retries",
        "2",
        "--use-waypoint-yaw",
    ]


def test_debug_bag_args_reject_unknown_profile_and_unsafe_waypoint():
    with pytest.raises(ValueError, match="unknown debug profile"):
        launch_manager.build_debug_bag_args(
            "/tmp/wrapper.sh",
            allowed_profiles=["zt_0"],
            profile_waypoint_files={},
            request={"profile": "bad"},
        )

    with pytest.raises(ValueError, match="target_id"):
        launch_manager.build_debug_bag_args(
            "/tmp/wrapper.sh",
            allowed_profiles=["zt_0"],
            profile_waypoint_files={},
            request={"profile": "zt_0", "target_id": "POI_001;reboot"},
        )


def test_start_navigation_headless_supports_legacy_profile_and_rviz_env_override():
    script_text = START_HEADLESS_SCRIPT.read_text(encoding="utf-8")

    assert "GO2W_HEADLESS_USE_RVIZ" in script_text
    assert "legacy positional profile" in script_text


def test_phone_ui_contains_debug_bag_controls_and_rviz_link():
    html = WEB_INDEX_PATH.read_text(encoding="utf-8")

    assert "/api/debug/start" in html
    assert "/api/debug/status" in html
    assert "/api/waypoints?profile=" in html
    assert "debugNearGoalDistance" in html
    assert "debugScanMinHz" in html
    assert "rvizLink" in html


def test_signal_handler_stops_services_before_async_server_shutdown():
    source = LAUNCH_MANAGER_PATH.read_text(encoding="utf-8")

    assert 'name="launch-manager-shutdown"' in source
    assert "target=server.shutdown" in source
