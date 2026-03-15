"""
GO2W real robot navigation using OFFICIAL Unitree SLAM (no slam_toolbox).

Pipeline:
  official SLAM topics from robot -> odom/tf bridge + scan conversion -> Nav2
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")
    monitor_pkg_dir = get_package_share_directory("go2w_office_sim")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_cmd_bridge = LaunchConfiguration("enable_cmd_bridge")

    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_real.urdf.xacro")
    nav2_params_file = os.path.join(pkg_dir, "config", "nav2_params_official_slam.yaml")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    rviz_config_file = os.path.join(monitor_pkg_dir, "rviz", "go2w_official_slam_status.rviz")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="",
            param_rewrites={"autostart": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_net_iface = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface connected to GO2W (for SDK command bridge)",
    )
    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    declare_enable_cmd_bridge = DeclareLaunchArgument(
        "enable_cmd_bridge",
        default_value="true",
        description="Start go2w_bridge.py for /cmd_vel -> SportClient command output",
    )

    stdout_linebuf = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", xacro_file]),
                    value_type=str,
                ),
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    # Keep this node for cmd_vel -> SportClient.Move.
    # Remap odom output to avoid conflicting with official SLAM odom bridge.
    go2w_bridge = Node(
        package="go2w_real",
        executable="go2w_bridge.py",
        name="go2w_bridge",
        output="screen",
        condition=IfCondition(enable_cmd_bridge),
        remappings=[
            ("odom", "/sport_odom"),
            ("imu/data", "/sport_imu"),
        ],
        parameters=[
            {
                "network_interface": network_interface,
                "cmd_vel_timeout": 0.5,
                "odom_frame": "odom",
                "base_frame": "base",
                "publish_odom_tf": False,
            }
        ],
    )

    official_odom_bridge = Node(
        package="go2w_real",
        executable="official_slam_odom_bridge.py",
        name="official_slam_odom_bridge",
        output="screen",
        parameters=[
            {
                "input_odom_topic": "/unitree/slam_mapping/odom",
                "output_odom_topic": "/odom",
                "frame_id_override": "map",
                "child_frame_id_override": "lidar",
                "publish_tf": True,
            }
        ],
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            {
                "use_sim_time": False,
                "target_frame": "lidar",
                "min_height": -0.2,
                "max_height": 0.4,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00872665,
                "scan_time": 0.1,
                "range_min": 0.05,
                "range_max": 30.0,
                "inf_epsilon": 1.0,
                "use_inf": True,
            }
        ],
        remappings=[
            ("cloud_in", "/unitree/slam_lidar/points"),
            ("scan", "/scan_raw"),
        ],
        output="screen",
    )

    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_file],
        remappings=[
            ("scan", "/scan_raw"),
            ("scan_filtered", "/scan"),
        ],
        output="screen",
    )

    # Optional live status marker/path for RViz overlay
    status_node = Node(
        package="go2w_office_sim",
        executable="go2w_official_slam_status_node.py",
        name="go2w_official_slam_status",
        output="screen",
    )

    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", False),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": nav2_lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )

    info = LogInfo(
        msg="\n\n========================================\n"
        "  GO2W Real Robot - OFFICIAL SLAM + Nav2\n"
        "  - slam_toolbox is NOT used in this launch.\n"
        "  - Ensure official SLAM is already running on robot.\n"
        "  - Required topics: /global_map /unitree/slam_mapping/odom /unitree/slam_lidar/points\n"
        "========================================\n"
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_net_iface,
            declare_use_rviz,
            declare_enable_cmd_bridge,
            robot_state_publisher,
            go2w_bridge,
            official_odom_bridge,
            pointcloud_to_scan,
            scan_filter,
            status_node,
            TimerAction(period=4.0, actions=[nav2_nodes]),
            rviz_node,
            info,
        ]
    )
