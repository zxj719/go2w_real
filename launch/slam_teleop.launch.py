"""
Backward-compatible alias for slam_mapping.launch.py.

This keeps older commands working while forwarding to the current
remote-mapping launch.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")
    target_launch = os.path.join(pkg_dir, "launch", "slam_mapping.launch.py")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    odom_source_mode = LaunchConfiguration("odom_source_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("network_interface", default_value="eth0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "cloud_topic", default_value="/unitree/slam_lidar/points"
            ),
            DeclareLaunchArgument("lidar_yaw_offset", default_value="1.5708"),
            DeclareLaunchArgument(
                "rviz_software_rendering", default_value="true"
            ),
            DeclareLaunchArgument("odom_source_mode", default_value="hybrid"),
            LogInfo(
                msg="[go2w_real] `slam_teleop.launch.py` is deprecated; "
                "forwarding to `slam_mapping.launch.py`."
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(target_launch),
                launch_arguments={
                    "network_interface": network_interface,
                    "use_rviz": use_rviz,
                    "cloud_topic": cloud_topic,
                    "lidar_yaw_offset": lidar_yaw_offset,
                    "rviz_software_rendering": rviz_software_rendering,
                    "odom_source_mode": odom_source_mode,
                }.items(),
            ),
        ]
    )
