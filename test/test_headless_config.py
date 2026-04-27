from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2w_real.headless_config import (
    DEFAULT_HEADLESS_CONFIG_PATH,
    HeadlessConfigError,
    build_executor_settings,
    build_localization_param_rewrites,
    build_nav2_param_rewrites,
    load_headless_config,
    resolve_headless_profile,
)



def test_default_headless_config_resolves_zt0_profile():
    config = load_headless_config(DEFAULT_HEADLESS_CONFIG_PATH)
    profile = resolve_headless_profile(config, None)

    assert profile["name"] == "zt_0"
    assert profile["waypoint_file"].endswith("go2w_map_waypoints.yaml")
    assert profile["slam_map_file"].endswith("/map/zt_0")


def test_headless_config_builds_nav2_and_localization_rewrites():
    config = load_headless_config(DEFAULT_HEADLESS_CONFIG_PATH)

    assert build_nav2_param_rewrites(config) == {
        "xy_goal_tolerance": "0.5",
        "yaw_goal_tolerance": "3.14",
    }
    assert build_localization_param_rewrites(config) == {
        "do_loop_closing": "true",
        "link_match_minimum_response_fine": "0.2",
        "loop_match_minimum_response_coarse": "0.5",
        "loop_match_minimum_response_fine": "0.6",
    }


def test_headless_localization_rewrite_defaults_keep_loop_closing_enabled():
    assert build_localization_param_rewrites({}) == {
        "do_loop_closing": "true",
        "link_match_minimum_response_fine": "0.2",
        "loop_match_minimum_response_coarse": "0.5",
        "loop_match_minimum_response_fine": "0.6",
    }


def test_headless_config_builds_executor_settings_from_selected_profile():
    config = load_headless_config(DEFAULT_HEADLESS_CONFIG_PATH)
    executor = build_executor_settings(config, "test_1")

    assert executor["profile"] == "test_1"
    assert (
        executor["server_uri"]
        == "ws://192.168.123.186:8100/ws/navigation/executor"
    )
    assert executor["waypoint_file"].endswith("go2w_waypoints.yaml")
    assert executor["show_logs"] is True
    assert executor["record_bag"] is True
    assert executor["use_odom_fusion"] is True


def test_headless_config_builds_executor_settings_from_14_1_profile():
    config = load_headless_config(DEFAULT_HEADLESS_CONFIG_PATH)
    executor = build_executor_settings(config, "14_1")

    assert executor["profile"] == "14_1"
    assert executor["waypoint_file"].endswith("go2w_waypoints.yaml")
    assert executor["slam_map_file"].endswith("/map/14_1")


def test_resolve_headless_profile_rejects_unknown_profile(tmp_path):
    config_path = tmp_path / "navigation_headless.yaml"
    config_path.write_text(
        "default_profile: zt_0\nprofiles:\n  zt_0:\n    waypoint_file: /tmp/a.yaml\n    slam_map_file: /tmp/a\n",
        encoding="utf-8",
    )

    config = load_headless_config(config_path)

    with pytest.raises(HeadlessConfigError, match="unknown headless profile"):
        resolve_headless_profile(config, "does_not_exist")
