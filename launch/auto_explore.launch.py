#!/usr/bin/env python3
"""
GO2W autonomous frontier exploration launch.

This is an **independent** launch that starts only the frontier explorer node.
It assumes the full SLAM + Nav2 stack is already running (e.g. via slam_rf2o.launch.py).

Usage:
  # Terminal 1 — bring up robot + SLAM + Nav2:
  ros2 launch go2w_real slam_rf2o.launch.py network_interface:=eth0

  # Terminal 2 — start autonomous exploration:
  ros2 launch go2w_real auto_explore.launch.py

  # Pause / resume exploration at any time:
  ros2 topic pub /frontier_explorer/explore/resume std_msgs/Bool '{data: false}' --once
  ros2 topic pub /frontier_explorer/explore/resume std_msgs/Bool '{data: true}' --once
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")

    explorer_params_file = LaunchConfiguration("explorer_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")

    frontier_explorer = TimerAction(
        period=LaunchConfiguration("start_delay"),
        actions=[
            Node(
                package="go2w_real",
                executable="frontier_explorer_cpp",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    explorer_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "explorer_params_file",
                default_value=os.path.join(
                    pkg_dir, "config", "frontier_explorer_params.yaml"
                ),
                description="Frontier explorer parameters file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="3.0",
                description="Seconds to wait before starting the explorer "
                "(gives Nav2 time to finish lifecycle transitions)",
            ),
            frontier_explorer,
        ]
    )
