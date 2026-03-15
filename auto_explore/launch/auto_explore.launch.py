"""
GO2W autonomous frontier exploration using SLAM Toolbox + Nav2 (Foxy).

Pipeline:
  Robot (Jetson): xt16_driver must be running for LiDAR pointcloud
  All nodes run on robot:
    go2w_bridge             -> odom + cmd_vel (via SportClient)
    static TF publishers    -> base->body->lidar->rslidar
    pointcloud_to_laserscan -> /scan_raw
    laser_filters           -> /scan
    SLAM Toolbox            -> /map + map->odom TF
    Nav2 (Foxy)             -> DWB controller + SmacPlanner2D
    frontier_explorer       -> autonomous exploration goals

Requires: xt16_driver running on robot for /unitree/slam_lidar/points
Does NOT require unitree_slam binary (SLAM Toolbox replaces it).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_auto_explore")
    real_pkg_dir = get_package_share_directory("go2w_real")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")

    # Config files
    nav2_params_file = os.path.join(pkg_dir, "config", "nav2_params_foxy.yaml")
    slam_params_file = os.path.join(real_pkg_dir, "config", "slam_params.yaml")
    laser_filter_file = os.path.join(real_pkg_dir, "config", "laser_filter.yaml")
    explore_params_file = os.path.join(pkg_dir, "config", "explore_params.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "auto_explore.rviz")

    # Nav2 params with autostart (Foxy: no ParameterFile wrapper)
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={"autostart": "true"},
        convert_types=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ── Launch arguments ──────────────────────────────────────────────
    declare_net_iface = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface connected to GO2W (123 segment)",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true"
    )
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/unitree/slam_lidar/points",
        description="PointCloud2 topic from LiDAR driver",
    )
    declare_lidar_yaw = DeclareLaunchArgument(
        "lidar_yaw_offset",
        default_value="1.5708",
        description="Yaw rotation (rad) from lidar frame to rslidar frame. "
        "xt16_driver may have different axis convention than unilidar SDK. "
        "Try 1.5708 (90°) or -1.5708 (-90°) if heading is misaligned.",
    )

    stdout_linebuf = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # ── 1. Static TFs: base->body (identity), body->lidar ────────────
    # Replaces full URDF robot_state_publisher to avoid go2w_office_sim
    # mesh dependency on the robot. Only the TF chain matters for SLAM/Nav2.
    base_to_body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_body",
        arguments=["0", "0", "0", "0", "0", "0", "base", "body"],
        output="screen",
    )
    base_to_base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_base_footprint",
        arguments=["0", "0", "0", "0", "0", "0", "base", "base_footprint"],
        output="screen",
    )
    body_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_lidar",
        arguments=["0.28945", "0", "-0.046825", "0", "0", "0", "body", "lidar"],
        output="screen",
    )

    # ── 2. GO2W Bridge (odom + cmd_vel via SportClient) ──────────────
    go2w_bridge = Node(
        package="go2w_real",
        executable="go2w_bridge.py",
        name="go2w_bridge",
        output="screen",
        parameters=[
            {
                "network_interface": network_interface,
                "cmd_vel_timeout": 0.5,
                "odom_frame": "odom",
                "base_frame": "base",
                "publish_odom_tf": True,
            }
        ],
    )

    # ── 3. Static TF: lidar -> rslidar ────────────────────────────────
    # xt16_driver pointcloud uses frame_id='rslidar'; URDF uses 'lidar'.
    # The xt16_driver may have a different X-axis convention than the
    # unilidar SDK. The lidar_yaw_offset parameter corrects for this.
    # static_transform_publisher args: x y z yaw pitch roll parent child
    rslidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_to_rslidar",
        arguments=["0", "0", "0", lidar_yaw_offset, "0", "0", "lidar", "rslidar"],
        output="screen",
    )

    # ── 4. Scan pipeline ─────────────────────────────────────────────
    # QoS relay: xt16_driver publishes RELIABLE, but pointcloud_to_laserscan
    # subscribes BEST_EFFORT (SensorDataQoS). CycloneDDS won't deliver
    # RELIABLE -> BEST_EFFORT. This relay bridges the QoS gap.
    pointcloud_relay = Node(
        package="go2w_auto_explore",
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

    # ── 5. SLAM Toolbox (online async) ───────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # ── 6. Nav2 Stack (Foxy-compatible) ──────────────────────────────
    # Foxy Nav2: no velocity_smoother, no collision_monitor, no smoother_server
    # Uses nav2_recoveries (not nav2_behaviors), DWB controller (not MPPI)
    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "recoveries_server",
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
                package="nav2_recoveries",
                executable="recoveries_server",
                name="recoveries_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
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

    # ── 7. Frontier Explorer C++ (delayed for SLAM + Nav2 startup) ───
    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="go2w_auto_explore",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[explore_params_file],
            ),
        ],
    )

    # Wait for odom + static TF chain to be available before starting SLAM.
    # Without this, slam_toolbox can start before go2w_bridge has published
    # odom->base, causing dropped scans at startup.
    wait_for_odom_to_lidar_tf = Node(
        package="go2w_auto_explore",
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

    # ── 8. RViz (optional) ───────────────────────────────────────────
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
        "  GO2W Auto Explore — SLAM Toolbox + Nav2 (Foxy)\n"
        "  - Ensure xt16_driver running on robot\n"
        "  - No unitree_slam needed (SLAM Toolbox replaces it)\n"
        "  - Robot must enter sport mode within 30s for /odom + TF\n"
        "  - Stand:  ros2 topic pub /cmd_control std_msgs/String"
        " '{data: stand_up}' --once\n"
        "========================================\n"
    )

    def launch_stack_after_tf_gate(event, _context):
        if event.returncode == 0:
            return [
                slam_toolbox,
                TimerAction(period=10.0, actions=[nav2_nodes]),
                frontier_explorer,
            ]

        return [
            LogInfo(
                msg="[auto_explore] TF gate failed: odom <- lidar was not "
                "available within 30s. Not starting slam_toolbox/Nav2."
            )
        ]

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_net_iface,
            declare_use_rviz,
            declare_cloud_topic,
            declare_lidar_yaw,
            SetParameter("use_sim_time", False),
            base_to_body_tf,
            base_to_base_footprint_tf,
            body_to_lidar_tf,
            go2w_bridge,
            rslidar_tf,
            pointcloud_relay,
            pointcloud_to_scan,
            scan_filter,
            wait_for_odom_to_lidar_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_odom_to_lidar_tf,
                    on_exit=launch_stack_after_tf_gate,
                )
            ),
            rviz_node,
            info,
        ]
    )
