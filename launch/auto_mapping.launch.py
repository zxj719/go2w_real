#!/usr/bin/env python3
"""
GO2W single-command autonomous mapping entrypoint.

This launch starts:
  pointcloud -> scan -> rf2o_laser_odometry -> slam_toolbox(mapping) -> Nav2
  -> frontier explorer

It still assumes:
  - the XT16 lidar driver is already publishing /unitree/slam_lidar/points
  - the robot is standing
  - the ROS environment has been sourced

Usage:
  ros2 launch go2w_real auto_mapping.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")
    slam_launch_file = os.path.join(pkg_dir, "launch", "slam_rf2o.launch.py")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    explorer_params_file = LaunchConfiguration("explorer_params_file")
    explorer_start_delay = LaunchConfiguration("explorer_start_delay")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "network_interface",
                default_value="eth0",
                description="Network interface connected to GO2W",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz while autonomous mapping runs",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/unitree/slam_lidar/points",
                description="PointCloud2 topic used for scan conversion",
            ),
            DeclareLaunchArgument(
                "lidar_yaw_offset",
                default_value="1.5708",
                description="Yaw rotation (rad) from lidar frame to rslidar frame",
            ),
            DeclareLaunchArgument(
                "rviz_software_rendering",
                default_value="false",
                description="Force RViz to use software OpenGL rendering if needed",
            ),
            DeclareLaunchArgument(
                "explorer_params_file",
                default_value=os.path.join(
                    pkg_dir, "config", "frontier_explorer_params.yaml"
                ),
                description="Frontier explorer parameters file",
            ),
            DeclareLaunchArgument(
                "explorer_start_delay",
                default_value="8.0",
                description="Seconds to wait after Nav2 launch before exploring",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={
                    "network_interface": network_interface,
                    "use_rviz": use_rviz,
                    "cloud_topic": cloud_topic,
                    "lidar_yaw_offset": lidar_yaw_offset,
                    "rviz_software_rendering": rviz_software_rendering,
                    "slam_mode": "mapping",
                    "start_explorer": "true",
                    "explorer_params_file": explorer_params_file,
                    "explorer_start_delay": explorer_start_delay,
                }.items(),
            ),
        ]
    )
