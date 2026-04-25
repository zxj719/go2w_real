"""
GO2W real robot SLAM + Nav2 with RF2O-first local odometry:
  pointcloud -> scan -> RF2O -> /odom
  optional: /sport_odom + /sport_imu -> robot_localization EKF -> /odom
  pointcloud -> scan -> slam_toolbox -> Nav2

Supports two SLAM modes:
  - localization: continue from a serialized slam_toolbox map prefix
  - mapping: start a fresh online async map

Optionally starts the frontier explorer after Nav2 is online.

Usage:
  ros2 launch go2w_real slam_rf2o.launch.py network_interface:=eth0

This launch assumes:
  - the robot is already standing
  - lidar points are available on /unitree/slam_lidar/points
  - RViz is optional
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

from go2w_real.headless_config import (
    DEFAULT_HEADLESS_CONFIG_PATH,
    build_localization_param_rewrites,
    build_nav2_param_rewrites,
    load_headless_config,
)


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_real")

    network_interface = LaunchConfiguration("network_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_yaw_offset = LaunchConfiguration("lidar_yaw_offset")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    slam_mode = LaunchConfiguration("slam_mode")
    slam_map_file = LaunchConfiguration("slam_map_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_localization_params_file = LaunchConfiguration(
        "slam_localization_params_file"
    )
    slam_mapping_params_file = LaunchConfiguration("slam_mapping_params_file")
    angular_deadband = LaunchConfiguration("angular_deadband")
    use_odom_fusion = LaunchConfiguration("use_odom_fusion")
    odom_fusion_params_file = LaunchConfiguration("odom_fusion_params_file")
    enable_motion_executor = LaunchConfiguration("enable_motion_executor")
    motion_executor_mode = LaunchConfiguration("motion_executor_mode")
    start_explorer = LaunchConfiguration("start_explorer")
    explorer_params_file = LaunchConfiguration("explorer_params_file")
    explorer_start_delay = LaunchConfiguration("explorer_start_delay")
    headless_config_file = LaunchConfiguration("headless_config_file")
    headless_profile = LaunchConfiguration("headless_profile")
    headless_nav2_xy_goal_tolerance = LaunchConfiguration(
        "headless_nav2_xy_goal_tolerance"
    )
    headless_nav2_yaw_goal_tolerance = LaunchConfiguration(
        "headless_nav2_yaw_goal_tolerance"
    )
    headless_do_loop_closing = LaunchConfiguration("headless_do_loop_closing")
    headless_link_match_minimum_response_fine = LaunchConfiguration(
        "headless_link_match_minimum_response_fine"
    )
    headless_loop_match_minimum_response_coarse = LaunchConfiguration(
        "headless_loop_match_minimum_response_coarse"
    )
    headless_loop_match_minimum_response_fine = LaunchConfiguration(
        "headless_loop_match_minimum_response_fine"
    )

    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_real.urdf.xacro")
    default_slam_localization_params_file = os.path.join(
        pkg_dir, "config", "slam_params.yaml"
    )
    default_slam_mapping_params_file = os.path.join(
        pkg_dir, "config", "slam_mapping_params.yaml"
    )
    default_nav2_params_file = os.path.join(
        pkg_dir, "config", "nav2_params_foxy.yaml"
    )
    odom_fusion_default_params_file = os.path.join(
        pkg_dir, "config", "odom_fusion_params.yaml"
    )
    motion_executor_params_file = os.path.join(
        pkg_dir, "config", "go2w_motion_executor.yaml"
    )
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    default_explorer_params_file = os.path.join(
        pkg_dir, "config", "frontier_explorer_params.yaml"
    )
    rviz_config_file = os.path.join(pkg_dir, "rviz", "nav2_real.rviz")
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "autostart": "true",
            "xy_goal_tolerance": headless_nav2_xy_goal_tolerance,
            "yaw_goal_tolerance": headless_nav2_yaw_goal_tolerance,
        },
        convert_types=True,
    )
    configured_localization_slam_params = RewrittenYaml(
        source_file=slam_localization_params_file,
        root_key="",
        param_rewrites={
            "mode": "localization",
            "map_file_name": slam_map_file,
            "map_start_at_dock": "true",
            "do_loop_closing": headless_do_loop_closing,
            "link_match_minimum_response_fine": headless_link_match_minimum_response_fine,
            "loop_match_minimum_response_coarse": headless_loop_match_minimum_response_coarse,
            "loop_match_minimum_response_fine": headless_loop_match_minimum_response_fine,
        },
        convert_types=True,
    )
    configured_mapping_slam_params = RewrittenYaml(
        source_file=slam_mapping_params_file,
        root_key="",
        param_rewrites={
            "mode": "mapping",
        },
        convert_types=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_net_iface = DeclareLaunchArgument(
        "network_interface",
        default_value=EnvironmentVariable(
            "GO2W_NETWORK_INTERFACE",
            default_value="eth0",
        ),
        description="Network interface connected to GO2W",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for SLAM + Nav2 goal testing",
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
        default_value="false",
        description="Force RViz to use software OpenGL rendering if hardware GL is unstable",
    )
    declare_slam_mode = DeclareLaunchArgument(
        "slam_mode",
        default_value="localization",
        description="slam_toolbox mode: localization or mapping",
    )
    declare_slam_map_file = DeclareLaunchArgument(
        "slam_map_file",
        default_value=EnvironmentVariable(
            "GO2W_SLAM_MAP_FILE",
            default_value="/home/unitree/ros_ws/src/map/",
        ),
        description=(
            "Serialized slam_toolbox map prefix without .data/.posegraph suffix"
        ),
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=default_nav2_params_file,
        description="Nav2 parameter YAML file",
    )
    declare_slam_localization_params_file = DeclareLaunchArgument(
        "slam_localization_params_file",
        default_value=default_slam_localization_params_file,
        description="slam_toolbox localization parameter YAML file",
    )
    declare_slam_mapping_params_file = DeclareLaunchArgument(
        "slam_mapping_params_file",
        default_value=default_slam_mapping_params_file,
        description="slam_toolbox mapping parameter YAML file",
    )
    declare_angular_deadband = DeclareLaunchArgument(
        "angular_deadband",
        default_value="0.12",
        description=(
            "Clamp cmd_vel angular.z to zero when its magnitude is below this threshold"
        ),
    )
    declare_use_odom_fusion = DeclareLaunchArgument(
        "use_odom_fusion",
        default_value="false",
        description=(
            "Default false keeps /odom on RF2O odom only; set true to publish "
            "/odom from robot_localization EKF fused from /sport_odom + /sport_imu"
        ),
    )
    declare_odom_fusion_params_file = DeclareLaunchArgument(
        "odom_fusion_params_file",
        default_value=odom_fusion_default_params_file,
        description="robot_localization EKF parameters for fused /odom output",
    )
    declare_enable_motion_executor = DeclareLaunchArgument(
        "enable_motion_executor",
        default_value="false",
        description=(
            "Enable GO2W segmented motion execution between Nav2 and the bridge"
        ),
    )
    declare_motion_executor_mode = DeclareLaunchArgument(
        "motion_executor_mode",
        default_value="shadow",
        description="Runtime mode for go2w_motion_executor: shadow or active",
    )
    declare_start_explorer = DeclareLaunchArgument(
        "start_explorer",
        default_value="false",
        description="Start the frontier explorer after Nav2 becomes active",
    )
    declare_explorer_params_file = DeclareLaunchArgument(
        "explorer_params_file",
        default_value=default_explorer_params_file,
        description="Frontier explorer parameters file",
    )
    declare_explorer_start_delay = DeclareLaunchArgument(
        "explorer_start_delay",
        default_value="8.0",
        description="Seconds to wait after Nav2 launch before starting exploration",
    )
    declare_headless_config_file = DeclareLaunchArgument(
        "headless_config_file",
        default_value=str(DEFAULT_HEADLESS_CONFIG_PATH),
        description="Unified headless runtime config YAML",
    )
    declare_headless_profile = DeclareLaunchArgument(
        "headless_profile",
        default_value="",
        description="Unified headless runtime profile name",
    )

    def _load_headless_runtime_overrides(context):
        config_path = headless_config_file.perform(context)
        config = load_headless_config(config_path)
        nav2_rewrites = build_nav2_param_rewrites(config)
        localization_rewrites = build_localization_param_rewrites(config)
        _ = headless_profile.perform(context)

        return [
            SetLaunchConfiguration(
                "headless_nav2_xy_goal_tolerance",
                nav2_rewrites["xy_goal_tolerance"],
            ),
            SetLaunchConfiguration(
                "headless_nav2_yaw_goal_tolerance",
                nav2_rewrites["yaw_goal_tolerance"],
            ),
            SetLaunchConfiguration(
                "headless_do_loop_closing",
                localization_rewrites["do_loop_closing"],
            ),
            SetLaunchConfiguration(
                "headless_link_match_minimum_response_fine",
                localization_rewrites["link_match_minimum_response_fine"],
            ),
            SetLaunchConfiguration(
                "headless_loop_match_minimum_response_coarse",
                localization_rewrites["loop_match_minimum_response_coarse"],
            ),
            SetLaunchConfiguration(
                "headless_loop_match_minimum_response_fine",
                localization_rewrites["loop_match_minimum_response_fine"],
            ),
        ]

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
        remappings=[
            ("odom", "/sport_odom"),
            ("imu/data", "/sport_imu"),
        ],
        parameters=[
            {
                "cmd_vel_timeout": 0.5,
                "angular_deadband": angular_deadband,
                "odom_frame": "odom",
                "base_frame": "base",
                "publish_odom_tf": False,
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

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                # Keep RF2O slightly below the 10 Hz scan pipeline to avoid
                # spinning idle cycles that only print "Waiting for laser_scans....".
                "freq": 9.0,
            }
        ],
        condition=UnlessCondition(use_odom_fusion),
    )

    odom_fusion_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[odom_fusion_params_file],
        remappings=[("odometry/filtered", "/odom")],
        condition=IfCondition(use_odom_fusion),
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

    wait_for_map_to_lidar_tf = Node(
        package="go2w_real",
        executable="wait_for_transform.py",
        name="wait_for_map_to_lidar_tf",
        output="screen",
        parameters=[
            {
                "target_frame": "map",
                "source_frame": "lidar",
                "timeout_sec": 60.0,
                "poll_period": 0.2,
                "log_period": 2.0,
            }
        ],
    )

    mapping_slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            configured_mapping_slam_params,
            {"use_sim_time": False},
        ],
        remappings=remappings,
        condition=IfCondition(
            PythonExpression(["'", slam_mode, "' == 'mapping'"])
        ),
    )

    localization_slam_toolbox = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            configured_localization_slam_params,
            {"use_sim_time": False},
        ],
        remappings=remappings,
        condition=IfCondition(
            PythonExpression(["'", slam_mode, "' == 'localization'"])
        ),
    )

    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "recoveries_server",
        "bt_navigator",
    ]

    lifecycle_manager_navigation = Node(
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
    )

    motion_executor_active_enabled = IfCondition(
        PythonExpression(
            [
                "'",
                enable_motion_executor,
                "' == 'true' and '",
                motion_executor_mode,
                "' == 'active'",
            ]
        )
    )
    motion_executor_shadow_or_disabled = UnlessCondition(
        PythonExpression(
            [
                "'",
                enable_motion_executor,
                "' == 'true' and '",
                motion_executor_mode,
                "' == 'active'",
            ]
        )
    )

    go2w_motion_executor = Node(
        package="go2w_real",
        executable="go2w_motion_executor.py",
        name="go2w_motion_executor",
        output="screen",
        parameters=[
            motion_executor_params_file,
            {"executor_mode": motion_executor_mode},
        ],
        condition=IfCondition(enable_motion_executor),
    )

    nav2_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", False),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
                condition=motion_executor_shadow_or_disabled,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "/nav2_cmd_vel_raw")],
                condition=motion_executor_active_enabled,
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
                condition=motion_executor_shadow_or_disabled,
            ),
            Node(
                package="nav2_recoveries",
                executable="recoveries_server",
                name="recoveries_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "/nav2_recovery_cmd_vel")],
                condition=motion_executor_active_enabled,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            go2w_motion_executor,
            # Delay lifecycle_manager slightly so the lifecycle servers are
            # already discoverable when autostart kicks in. Without this,
            # Foxy occasionally leaves the whole Nav2 stack unconfigured.
            TimerAction(
                period=2.0,
                actions=[lifecycle_manager_navigation],
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

    frontier_explorer = TimerAction(
        period=explorer_start_delay,
        condition=IfCondition(start_explorer),
        actions=[
            Node(
                package="go2w_real",
                executable="frontier_explorer_cpp",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    explorer_params_file,
                    {"use_sim_time": False},
                ],
            ),
        ],
    )

    def launch_stack_after_tf_gate(event, _context):
        if event.returncode == 0:
            return [mapping_slam_toolbox, localization_slam_toolbox]

        return [
            LogInfo(
                msg="[go2w_real] TF gate failed: odom <- lidar was not "
                "available within 30s. Not starting slam_toolbox/Nav2."
            )
        ]

    def launch_visualization_and_nav_after_map_tf_gate(event, _context):
        if event.returncode == 0:
            return [
                nav2_nodes,
                rviz_node,
                frontier_explorer,
            ]

        return [
            LogInfo(
                msg="[go2w_real] TF gate failed: map <- lidar was not "
                "available within 60s. Not starting RViz/Nav2."
            )
        ]

    info = LogInfo(
        msg=[
            "\n\n========================================\n"
            "  GO2W Real Robot - RF2O SLAM + Nav2\n"
            "  - slam_mode: ",
            slam_mode,
            "\n"
            "  - use_odom_fusion: ",
            use_odom_fusion,
            "\n"
            "  - enable_motion_executor: ",
            enable_motion_executor,
            "\n"
            "  - motion_executor_mode: ",
            motion_executor_mode,
            "\n"
            "  - start_explorer: ",
            start_explorer,
            "\n"
            "  - RViz opens with the Nav2 layout and a 2D Goal Pose tool\n"
            "  - Click '2D Goal Pose' in RViz, then drag to set goal position + yaw\n"
            "  - Robot must already be standing before motion commands are sent\n"
            "  - /odom defaults to RF2O odom only\n"
            "  - Set use_odom_fusion:=true to enable robot_localization EKF "
            "from /sport_odom + /sport_imu\n"
            "  - Pointcloud source defaults to /unitree/slam_lidar/points\n"
            "  - Save map: mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/go2w_map\n"
            "\n========================================\n",
        ]
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_net_iface,
            declare_use_rviz,
            declare_cloud_topic,
            declare_lidar_yaw_offset,
            declare_rviz_software_rendering,
            declare_slam_mode,
            declare_slam_map_file,
            declare_nav2_params_file,
            declare_slam_localization_params_file,
            declare_slam_mapping_params_file,
            declare_angular_deadband,
            declare_use_odom_fusion,
            declare_odom_fusion_params_file,
            declare_enable_motion_executor,
            declare_motion_executor_mode,
            declare_start_explorer,
            declare_explorer_params_file,
            declare_explorer_start_delay,
            declare_headless_config_file,
            declare_headless_profile,
            OpaqueFunction(function=_load_headless_runtime_overrides),
            rviz_software_gl,
            rviz_gl_version,
            robot_state_publisher,
            base_to_base_footprint_tf,
            lidar_to_rslidar_tf,
            go2w_bridge,
            pointcloud_relay,
            pointcloud_to_scan,
            scan_filter,
            rf2o_node,
            odom_fusion_node,
            wait_for_odom_to_lidar_tf,
            wait_for_map_to_lidar_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_odom_to_lidar_tf,
                    on_exit=launch_stack_after_tf_gate,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_map_to_lidar_tf,
                    on_exit=launch_visualization_and_nav_after_map_tf_gate,
                )
            ),
            info,
        ]
    )
