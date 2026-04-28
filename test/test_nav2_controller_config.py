import math
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_baseline_nav2_dwb_controller_targets():
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    controller_params = nav2_cfg["controller_server"]["ros__parameters"]
    goal_checker = controller_params["general_goal_checker"]
    progress_checker = controller_params["progress_checker"]
    follow_path = nav2_cfg["controller_server"]["ros__parameters"]["FollowPath"]

    assert math.isclose(
        goal_checker["xy_goal_tolerance"], 0.50, rel_tol=0.0, abs_tol=1e-6
    )
    assert math.isclose(
        goal_checker["yaw_goal_tolerance"], 3.14, rel_tol=0.0, abs_tol=1e-6
    )
    assert math.isclose(
        progress_checker["required_movement_radius"],
        0.05,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        progress_checker["movement_time_allowance"],
        12.0,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        controller_params["failure_tolerance"], 2.0, rel_tol=0.0, abs_tol=1e-6
    )
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert math.isclose(follow_path["max_vel_x"], 0.75, rel_tol=0.0, abs_tol=1e-6)
    assert math.isclose(follow_path["max_speed_xy"], 0.75, rel_tol=0.0, abs_tol=1e-6)
    assert math.isclose(
        follow_path["max_vel_theta"], 0.75, rel_tol=0.0, abs_tol=1e-6
    )
    assert math.isclose(follow_path["acc_lim_theta"], 1.2, rel_tol=0.0, abs_tol=1e-6)
    assert math.isclose(follow_path["decel_lim_theta"], -1.5, rel_tol=0.0, abs_tol=1e-6)
    assert follow_path["vx_samples"] == 15
    assert follow_path["vtheta_samples"] == 20
    assert math.isclose(follow_path["sim_time"], 1.6, rel_tol=0.0, abs_tol=1e-6)
    assert "Twirling" not in follow_path["critics"]
    assert math.isclose(follow_path["PathAlign.scale"], 12.0, rel_tol=0.0, abs_tol=1e-6)
    assert math.isclose(follow_path["GoalAlign.scale"], 8.0, rel_tol=0.0, abs_tol=1e-6)
    assert math.isclose(
        follow_path["RotateToGoal.scale"], 1.0, rel_tol=0.0, abs_tol=1e-6
    )
    assert math.isclose(
        follow_path["RotateToGoal.slowing_factor"], 5.0, rel_tol=0.0, abs_tol=1e-6
    )
    assert math.isclose(
        follow_path["transform_tolerance"], 2.0, rel_tol=0.0, abs_tol=1e-6
    )
    assert controller_params["goal_checker_plugin"] == "goal_checker"


def test_foxy_goal_checker_alias_matches_unified_goal_tolerances():
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    controller_params = nav2_cfg["controller_server"]["ros__parameters"]
    unified_goal_checker = controller_params["general_goal_checker"]
    foxy_goal_checker = controller_params["goal_checker"]

    assert foxy_goal_checker["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert foxy_goal_checker["stateful"] is True
    assert math.isclose(
        foxy_goal_checker["xy_goal_tolerance"],
        unified_goal_checker["xy_goal_tolerance"],
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        foxy_goal_checker["yaw_goal_tolerance"],
        unified_goal_checker["yaw_goal_tolerance"],
        rel_tol=0.0,
        abs_tol=1e-6,
    )


def test_nav2_real_hardware_tf_tolerances_are_relaxed_for_rf2o():
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    assert math.isclose(
        nav2_cfg["local_costmap"]["local_costmap"]["ros__parameters"][
            "transform_tolerance"
        ],
        2.0,
        rel_tol=0.0,
        abs_tol=1e-6,
    )


def test_costmaps_use_filtered_scan_for_dynamic_obstacles():
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    for costmap_name in ("local_costmap", "global_costmap"):
        params = nav2_cfg[costmap_name][costmap_name]["ros__parameters"]
        obstacle_layer = params["obstacle_layer"]

        assert obstacle_layer["observation_sources"] == "scan"
        assert obstacle_layer["scan"]["topic"] == "/scan"
        assert obstacle_layer["scan"]["marking"] is True
        assert obstacle_layer["scan"]["clearing"] is True
        assert math.isclose(
            obstacle_layer["scan"]["min_obstacle_height"],
            -0.20,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
        assert math.isclose(
            obstacle_layer["scan"]["obstacle_range"],
            4.0,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
        assert math.isclose(
            obstacle_layer["scan"]["raytrace_range"],
            4.5,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
        assert "obstacle_max_range" not in obstacle_layer["scan"]
        assert "raytrace_max_range" not in obstacle_layer["scan"]


def test_planner_tolerance_matches_goal_checker_xy_tolerance():
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    planner_tolerance = nav2_cfg["planner_server"]["ros__parameters"]["GridBased"][
        "tolerance"
    ]
    goal_checker_tolerance = nav2_cfg["controller_server"]["ros__parameters"][
        "general_goal_checker"
    ]["xy_goal_tolerance"]

    assert math.isclose(
        planner_tolerance,
        goal_checker_tolerance,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        nav2_cfg["global_costmap"]["global_costmap"]["ros__parameters"][
            "transform_tolerance"
        ],
        2.0,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        nav2_cfg["recoveries_server"]["ros__parameters"]["transform_tolerance"],
        2.0,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
