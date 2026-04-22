import math
from pathlib import Path

import yaml


REPO_ROOT = Path("/home/unitree/ros_ws")


def test_executor_config_matches_nav2_goal_tolerance_and_recovery_policy():
    executor_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/go2w_motion_executor.yaml").read_text()
    )
    nav2_cfg = yaml.safe_load(
        (REPO_ROOT / "src/go2w_real/config/nav2_params_foxy.yaml").read_text()
    )

    executor_params = executor_cfg["go2w_motion_executor"]["ros__parameters"]
    nav2_yaw = nav2_cfg["controller_server"]["ros__parameters"][
        "general_goal_checker"
    ]["yaw_goal_tolerance"]

    assert math.isclose(
        executor_params["shared_final_yaw_tolerance"],
        nav2_yaw,
        rel_tol=0.0,
        abs_tol=1e-6,
    )
    assert executor_params["navigation_status_topic"] == "/navigate_to_pose/_action/status"
    assert executor_params["costmap_unsafe_cost"] == 200
    assert nav2_cfg["recoveries_server"]["ros__parameters"]["recovery_plugins"] == [
        "backup",
        "spin",
        "wait",
    ]


def test_launch_file_can_optionally_route_nav2_through_executor():
    launch_text = (REPO_ROOT / "src/go2w_real/launch/slam_rf2o.launch.py").read_text()

    assert 'DeclareLaunchArgument(\n        "enable_motion_executor"' in launch_text
    assert 'default_value="false"' in launch_text
    assert 'DeclareLaunchArgument(\n        "motion_executor_mode"' in launch_text
    assert 'default_value="shadow"' in launch_text
    assert "go2w_motion_executor.py" in launch_text
    assert "motion_executor_params_file" in launch_text
    assert "motion_executor_active_enabled" in launch_text
    assert "motion_executor_shadow_or_disabled" in launch_text
    assert '"executor_mode": motion_executor_mode' in launch_text
    assert '("cmd_vel", "/nav2_cmd_vel_raw")' in launch_text
    assert '("cmd_vel", "/nav2_recovery_cmd_vel")' in launch_text


def test_motion_executor_config_exposes_shadow_cmd_output_topic():
    assert 'shadow_cmd_vel_topic: /go2w_motion_executor/shadow_cmd_vel' in (
        REPO_ROOT / "src/go2w_real/config/go2w_motion_executor.yaml"
    ).read_text()


def test_cmakelists_registers_all_executor_pytests():
    cmake_text = (REPO_ROOT / "src/go2w_real/CMakeLists.txt").read_text()

    assert "test_motion_executor_types" in cmake_text
    assert "test_motion_executor_geometry" in cmake_text
    assert "test_motion_executor_logic" in cmake_text
    assert "test_motion_executor_launch_config" in cmake_text


def test_launch_defaults_match_current_lidar_driver_contract():
    launch_text = (REPO_ROOT / "src/go2w_real/launch/slam_rf2o.launch.py").read_text()

    assert 'default_value="/unitree/slam_lidar/points"' in launch_text
    assert 'name="lidar_to_utlidar_lidar"' not in launch_text
    assert '"lidar", "utlidar_lidar"' not in launch_text


def test_launch_exposes_bridge_angular_deadband_argument():
    slam_launch_text = (
        REPO_ROOT / "src/go2w_real/launch/slam_rf2o.launch.py"
    ).read_text()
    bringup_launch_text = (
        REPO_ROOT / "src/go2w_real/launch/bringup.launch.py"
    ).read_text()

    assert 'DeclareLaunchArgument(\n        "angular_deadband"' in slam_launch_text
    assert 'default_value="0.1"' in slam_launch_text
    assert '"angular_deadband": angular_deadband' in slam_launch_text
    assert 'DeclareLaunchArgument(\n        "angular_deadband"' in bringup_launch_text
    assert 'default_value="0.1"' in bringup_launch_text
    assert '"angular_deadband": angular_deadband' in bringup_launch_text


def test_fast_recovery_tree_backs_up_before_spinning():
    bt_xml = (
        REPO_ROOT
        / "src/go2w_real/behavior_trees/navigate_w_replanning_and_recovery_fast.xml"
    ).read_text()

    assert (
        '<BackUp server_name="backup" backup_dist="0.20" backup_speed="0.08" server_timeout="5000"/>'
        in bt_xml
    )
    assert bt_xml.index("<BackUp") < bt_xml.index("<Spin")


def test_fast_recovery_tree_sets_explicit_server_timeout_on_bt_nodes():
    bt_xml = (
        REPO_ROOT
        / "src/go2w_real/behavior_trees/navigate_w_replanning_and_recovery_fast.xml"
    ).read_text()

    assert '<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_timeout="5000"/>' in bt_xml
    assert '<FollowPath path="{path}" controller_id="FollowPath" server_timeout="5000"/>' in bt_xml
    assert 'ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap" server_timeout="5000"/>' in bt_xml
    assert 'ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap" server_timeout="5000"/>' in bt_xml
    assert 'ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap" server_timeout="5000"/>' in bt_xml
    assert 'ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap" server_timeout="5000"/>' in bt_xml
    assert '<BackUp server_name="backup" backup_dist="0.20" backup_speed="0.08" server_timeout="5000"/>' in bt_xml
    assert '<Spin spin_dist="1.57" server_timeout="5000"/>' in bt_xml
    assert '<Wait wait_duration="2" server_timeout="5000"/>' in bt_xml
