"""
GO2W real robot bringup (teleop-ready):
  go2w_bridge + robot_state_publisher + laser_filter

Usage:
  ros2 launch go2w_real bringup.launch.py network_interface:=eth0
  # Then teleop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")

    network_interface = LaunchConfiguration("network_interface")
    angular_deadband = LaunchConfiguration("angular_deadband")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_real.urdf.xacro")

    # ===== Launch arguments =====
    declare_net_iface = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface connected to GO2W (e.g. eth0, enp2s0)",
    )
    declare_angular_deadband = DeclareLaunchArgument(
        "angular_deadband",
        default_value="0.12",
        description=(
            "Clamp cmd_vel angular.z to zero when its magnitude is below this threshold"
        ),
    )

    # ===== 1. Robot State Publisher =====
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

    # ===== 2. GO2W Bridge (SDK <-> ROS2) =====
    go2w_bridge = Node(
        package="go2w_real",
        executable="go2w_bridge.py",
        name="go2w_bridge",
        output="screen",
        arguments=[
            "--network-interface",
            network_interface,
        ],
        parameters=[
            {
                "cmd_vel_timeout": 0.5,
                "angular_deadband": angular_deadband,
                "odom_frame": "odom",
                "base_frame": "base",
                "publish_odom_tf": True,
            }
        ],
    )

    # ===== 3. 3D pointcloud -> 2D laser scan =====
    # Real GO2W L2 LiDAR publishes PointCloud2 on /unilidar/cloud
    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            {
                "use_sim_time": False,
                "target_frame": "lidar",
                "min_height": -0.1,
                "max_height": 0.3,
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
            ("cloud_in", "/unilidar/cloud"),
            ("scan", "/scan_raw"),
        ],
        output="screen",
    )

    # ===== 4. Laser scan filter (removes robot self-occlusion) =====
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

    info = LogInfo(
        msg="\n\n========================================\n"
        "  GO2W Real Robot Bringup\n"
        "  - Bridge node connects via SDK DDS\n"
        "  - Robot must already be standing before motion commands are sent\n"
        "  - Teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard\n"
        "  - Stand:  ros2 topic pub /cmd_control std_msgs/String '{data: stand_up}' --once\n"
        "  - Stop:   ros2 topic pub /cmd_control std_msgs/String '{data: stop}' --once\n"
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            declare_net_iface,
            declare_angular_deadband,
            robot_state_publisher,
            go2w_bridge,
            pointcloud_to_scan,
            scan_filter,
            info,
        ]
    )
