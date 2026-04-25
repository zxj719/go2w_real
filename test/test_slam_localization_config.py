from pathlib import Path

import yaml


REPO_ROOT = Path("/home/unitree/ros_ws")
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"


def test_localization_slam_params_use_conservative_matching():
    cfg = yaml.safe_load((PACKAGE_ROOT / "config/slam_params.yaml").read_text())
    params = cfg["slam_toolbox"]["ros__parameters"]

    assert params["do_loop_closing"] is False
    assert params["minimum_travel_distance"] == 0.1
    assert params["minimum_travel_heading"] == 0.02
    assert params["link_match_minimum_response_fine"] == 0.2
    assert params["loop_match_minimum_response_coarse"] == 0.5
    assert params["loop_match_minimum_response_fine"] == 0.6
