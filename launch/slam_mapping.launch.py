"""
GO2W real robot SLAM mapping:
  go2w_bridge + robot_state_publisher + lidar scan pipeline + slam_toolbox + RViz

Usage:
  ros2 launch go2w_real slam_mapping.launch.py network_interface:=eth0

This launch is intended for manual mapping with the Unitree remote:
  - robot stands up automatically via the Unitree Python SDK
  - slam_toolbox builds the occupancy grid map
  - driving is done externally, not by a ROS teleop node
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    odom_source_mode = LaunchConfiguration("odom_source_mode")

    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_real.urdf.xacro")
    slam_params_file = os.path.join(pkg_dir, "config", "slam_params.yaml")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "slam_mapping.rviz")

    declare_net_iface = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface connected to GO2W",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for SLAM mapping",
    )
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/unitree/slam_lidar/points",
        description="PointCloud2 topic used for scan conversion",
    )
    declare_lidar_yaw_offset = DeclareLaunchArgument(
        "lidar_yaw_offset",
        default_value="1.5708",
        description="Yaw rotation (rad) from lidar frame to rslidar frame",
    )
    declare_rviz_software_rendering = DeclareLaunchArgument(
        "rviz_software_rendering",
        default_value="true",
        description="Force RViz to use software OpenGL rendering",
    )
    declare_odom_source_mode = DeclareLaunchArgument(
        "odom_source_mode",
        default_value="hybrid",
        description="Bridge odom translation source: position, velocity, or hybrid",
    )

    stdout_linebuf = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    rviz_software_gl = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE",
        "1",
        condition=IfCondition(rviz_software_rendering),
    )
    rviz_gl_version = SetEnvironmentVariable(
        "MESA_GL_VERSION_OVERRIDE",
        "3.3",
        condition=IfCondition(rviz_software_rendering),
    )

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

    base_to_base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_base_footprint",
        arguments=["0", "0", "0", "0", "0", "0", "base", "base_footprint"],
        output="screen",
    )

    lidar_to_rslidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_to_rslidar",
        arguments=["0", "0", "0", lidar_yaw_offset, "0", "0", "lidar", "rslidar"],
        output="screen",
    )

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
                "odom_frame": "odom",
                "base_frame": "base",
                "publish_odom_tf": True,
                "odom_source_mode": odom_source_mode,
            }
        ],
    )

    pointcloud_relay = Node(
        package="go2w_real",
        executable="pointcloud_relay.py",
        name="pointcloud_relay",
        output="screen",
        parameters=[
            {
                "input_topic": cloud_topic,
                "output_topic": "/cloud_relayed",
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
            ("cloud_in", "/cloud_relayed"),
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

    wait_for_odom_to_lidar_tf = Node(
        package="go2w_real",
        executable="wait_for_transform.py",
        name="wait_for_odom_to_lidar_tf",
        output="screen",
        parameters=[
            {
                "target_frame": "odom",
                "source_frame": "lidar",
                "timeout_sec": 30.0,
                "poll_period": 0.2,
                "log_period": 2.0,
            }
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params_file,
        }.items(),
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

    def launch_slam_after_tf_gate(event, _context):
        if event.returncode == 0:
            return [slam_toolbox]

        return [
            LogInfo(
                msg="[go2w_real] TF gate failed: odom <- lidar was not "
                "available within 30s. Not starting slam_toolbox."
            )
        ]

    info = LogInfo(
        msg="\n\n========================================\n"
        "  GO2W Real Robot - SLAM Mapping\n"
        "  - Use the Unitree remote to drive while slam_toolbox builds /map\n"
        "  - Odom source mode defaults to hybrid (`odom_source_mode:=velocity` to force velocity integration)\n"
        "  - Pointcloud source defaults to /unitree/slam_lidar/points (`cloud_topic:=...` to override)\n"
        "  - Save map: mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/go2w_map\n"
        "  - Stand:    ros2 topic pub /cmd_control std_msgs/String '{data: stand_up}' --once\n"
        "  - Stop:     ros2 topic pub /cmd_control std_msgs/String '{data: stop}' --once\n"
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_net_iface,
            declare_use_rviz,
            declare_cloud_topic,
            declare_lidar_yaw_offset,
            declare_rviz_software_rendering,
            declare_odom_source_mode,
            rviz_software_gl,
            rviz_gl_version,
            robot_state_publisher,
            base_to_base_footprint_tf,
            lidar_to_rslidar_tf,
            go2w_bridge,
            pointcloud_relay,
            pointcloud_to_scan,
            scan_filter,
            wait_for_odom_to_lidar_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_odom_to_lidar_tf,
                    on_exit=launch_slam_after_tf_gate,
                )
            ),
            rviz_node,
            info,
        ]
    )
