"""
GO2W real robot SLAM mapping using lidar-only odometry:
  pointcloud -> scan -> rf2o_laser_odometry -> slam_toolbox + RViz

Usage:
  ros2 launch go2w_real slam_rf2o.launch.py

This launch assumes:
  - the robot is already standing
  - driving is done externally, e.g. with the Unitree remote
  - lidar points are available on /unitree/slam_lidar/points
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

    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")

    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_real.urdf.xacro")
    slam_params_file = os.path.join(pkg_dir, "config", "slam_params.yaml")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "slam_mapping.rviz")

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

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 12.0,
            }
        ],
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
        "  GO2W Real Robot - SLAM Mapping (RF2O)\n"
        "  - Use the Unitree remote to drive while slam_toolbox builds /map\n"
        "  - Odom is estimated from /scan by rf2o_laser_odometry\n"
        "  - Pointcloud source defaults to /unitree/slam_lidar/points\n"
        "  - Save map: mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/go2w_map\n"
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_use_rviz,
            declare_cloud_topic,
            declare_lidar_yaw_offset,
            declare_rviz_software_rendering,
            rviz_software_gl,
            rviz_gl_version,
            robot_state_publisher,
            base_to_base_footprint_tf,
            lidar_to_rslidar_tf,
            pointcloud_relay,
            pointcloud_to_scan,
            scan_filter,
            rf2o_node,
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
