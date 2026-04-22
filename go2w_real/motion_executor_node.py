from dataclasses import asdict, fields, replace
import math
from typing import Any, Callable, List, Optional

from .motion_executor_geometry import (
    GridCostmap2D,
    PathPoint2D,
    Pose2D,
    extract_straight_segment,
    is_segment_complete,
    segment_is_costmap_safe,
    segment_progress,
)
from .motion_executor_logic import (
    Command2D,
    CommandShaper,
    GoalStatusSnapshot,
    MeasuredMotion,
    MotionExecutorState,
    can_leave_drive,
    route_executor_command,
    should_accept_segment_completion,
    should_interrupt_drive_for_heading,
    should_stop_for_stale_path,
    should_switch_path,
)
from .motion_executor_types import GoalLifecycle, MotionExecutorParams, MotionPhase, StopReason

DEFAULT_NODE_PARAMETERS = {
    "executor_mode": "shadow",
    "shadow_cmd_vel_topic": "/go2w_motion_executor/shadow_cmd_vel",
    "path_topic": "/plan",
    "odom_topic": "/odom",
    "nav2_cmd_vel_topic": "/nav2_cmd_vel_raw",
    "nav2_recovery_cmd_vel_topic": "/nav2_recovery_cmd_vel",
    "local_costmap_topic": "/local_costmap/costmap",
    "navigation_status_topic": "/navigate_to_pose/_action/status",
}


def merged_parameter_defaults() -> dict:
    merged = dict(DEFAULT_NODE_PARAMETERS)
    merged.update(asdict(MotionExecutorParams()))
    return merged


_ROS_IMPORT_ERROR: Optional[ImportError] = None

try:
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import Point, PoseStamped, Twist
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Odometry, Path
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import Float64, String
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker
except ImportError as exc:  # pragma: no cover - depends on sourced ROS env.
    _ROS_IMPORT_ERROR = exc


def _ensure_ros_imports() -> None:
    if _ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS 2 Python dependencies are unavailable. Source /opt/ros/foxy/setup.bash "
            "before using go2w_real.motion_executor_node."
        ) from _ROS_IMPORT_ERROR


def _yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - (2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)),
    )


def _shortest_angular_distance(current: float, target: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _goal_snapshot_from_status_array(status_array: Any) -> GoalStatusSnapshot:
    if not status_array.status_list:
        return GoalStatusSnapshot(None, GoalLifecycle.NONE)

    latest = status_array.status_list[-1]
    goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
    if latest.status in (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    ):
        lifecycle = GoalLifecycle.ACTIVE
    elif latest.status == GoalStatus.STATUS_SUCCEEDED:
        lifecycle = GoalLifecycle.SUCCEEDED
    elif latest.status == GoalStatus.STATUS_CANCELED:
        lifecycle = GoalLifecycle.CANCELED
    elif latest.status == GoalStatus.STATUS_ABORTED:
        lifecycle = GoalLifecycle.ABORTED
    else:
        lifecycle = GoalLifecycle.NONE
    return GoalStatusSnapshot(goal_id, lifecycle)


def _transform_path_point(transform: Any, point: PathPoint2D) -> PathPoint2D:
    yaw = _yaw_from_quaternion(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return PathPoint2D(
        x=(cos_yaw * point.x) - (sin_yaw * point.y) + transform.transform.translation.x,
        y=(sin_yaw * point.x) + (cos_yaw * point.y) + transform.transform.translation.y,
        yaw=point.yaw + yaw,
    )


if _ROS_IMPORT_ERROR is None:

    class Go2WMotionExecutorNode(Node):
        def __init__(self) -> None:
            super().__init__("go2w_motion_executor")

            for name, value in merged_parameter_defaults().items():
                self.declare_parameter(name, value)

            self.params = MotionExecutorParams(
                **{
                    field.name: self.get_parameter(field.name).value
                    for field in fields(MotionExecutorParams)
                }
            )
            self.control_period = 0.05
            self.state = MotionExecutorState(self.params)
            self.shaper = CommandShaper(self.params)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.path_msg: Optional[Path] = None
            self.pending_path_msg: Optional[Path] = None
            self.last_path_received_at = None
            self.odom_msg: Optional[Odometry] = None
            self.costmap_msg: Optional[Costmap] = None
            self.last_nav2_cmd_observation: Optional[Twist] = None
            self.last_recovery_cmd = Twist()
            self.last_recovery_cmd_at = None
            self.last_robot_pose: Optional[Pose2D] = None
            self.drive_start_pose: Optional[Pose2D] = None
            self.active_segment = None
            self._warned_missing_costmap = False

            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.shadow_cmd_pub = self.create_publisher(
                Twist,
                self.get_parameter("shadow_cmd_vel_topic").value,
                10,
            )
            self.state_pub = self.create_publisher(
                String, "/go2w_motion_executor/state", 10
            )
            self.segment_pub = self.create_publisher(
                Marker, "/go2w_motion_executor/current_segment", 10
            )
            self.segment_path_pub = self.create_publisher(
                Path, "/go2w_motion_executor/segment_path", 10
            )
            self.yaw_error_pub = self.create_publisher(
                Float64, "/go2w_motion_executor/debug_yaw_error", 10
            )
            self.along_track_pub = self.create_publisher(
                Float64, "/go2w_motion_executor/debug_along_track_progress", 10
            )
            self.cross_track_pub = self.create_publisher(
                Float64, "/go2w_motion_executor/debug_cross_track_error", 10
            )
            self.goal_status_pub = self.create_publisher(
                String, "/go2w_motion_executor/goal_status", 10
            )
            self.stop_reason_pub = self.create_publisher(
                String, "/go2w_motion_executor/stop_reason", 10
            )

            self.create_subscription(
                Path,
                self.get_parameter("path_topic").value,
                self._on_path,
                10,
            )
            self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self._on_odom,
                20,
            )
            self.create_subscription(
                Twist,
                self.get_parameter("nav2_cmd_vel_topic").value,
                self._on_nav2_cmd,
                10,
            )
            self.create_subscription(
                Twist,
                self.get_parameter("nav2_recovery_cmd_vel_topic").value,
                self._on_recovery_cmd,
                10,
            )
            self.create_subscription(
                GoalStatusArray,
                self.get_parameter("navigation_status_topic").value,
                self._on_status,
                10,
            )
            self.create_subscription(
                Costmap,
                self.get_parameter("local_costmap_topic").value,
                self._on_costmap,
                10,
            )
            self.timer = self.create_timer(self.control_period, self._on_timer)

        def _on_path(self, msg: Path) -> None:
            received_at = self.get_clock().now()
            if (
                self.path_msg is None
                or self.state.phase != MotionPhase.DRIVE
                or self.active_segment is None
                or self.last_robot_pose is None
                or self.drive_start_pose is None
            ):
                self.path_msg = msg
                self.last_path_received_at = received_at
                return

            incoming_points = [
                PathPoint2D(
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    yaw=_yaw_from_quaternion(pose.pose.orientation),
                )
                for pose in msg.poses
            ]
            if len(incoming_points) < 2:
                self.pending_path_msg = msg
                return

            new_heading = math.atan2(
                incoming_points[1].y - incoming_points[0].y,
                incoming_points[1].x - incoming_points[0].x,
            )
            lateral_offset = math.hypot(
                incoming_points[0].x - self.active_segment.start.x,
                incoming_points[0].y - self.active_segment.start.y,
            )
            covers_continuation = len(incoming_points) > self.active_segment.end_index
            driven_distance = math.hypot(
                self.last_robot_pose.x - self.drive_start_pose.x,
                self.last_robot_pose.y - self.drive_start_pose.y,
            )

            should_take_path = should_switch_path(
                current_heading=self.active_segment.yaw,
                new_heading=new_heading,
                lateral_offset=lateral_offset,
                covers_continuation=covers_continuation,
                in_drive=True,
                params=self.params,
            )
            can_take_path = can_leave_drive(
                driven_distance,
                safety_required=False,
                params=self.params,
            )
            if should_take_path and can_take_path:
                self.path_msg = msg
                self.last_path_received_at = received_at
                self.pending_path_msg = None
                self.state.request_stop_for_phase(
                    MotionPhase.ROTATE, StopReason.PATH_REPLAN
                )
                return

            self.pending_path_msg = msg

        def _on_odom(self, msg: Odometry) -> None:
            self.odom_msg = msg

        def _on_nav2_cmd(self, msg: Twist) -> None:
            # Observation only. Nominal segmented motion decisions are derived
            # from /plan, TF pose, odom stillness, and executor state.
            self.last_nav2_cmd_observation = msg

        def _on_recovery_cmd(self, msg: Twist) -> None:
            self.last_recovery_cmd = msg
            self.last_recovery_cmd_at = self.get_clock().now()
            self.drive_start_pose = None
            self.shaper = CommandShaper(self.params)
            self.state.enter_recovery_passthrough()

        def _on_status(self, msg: GoalStatusArray) -> None:
            self.state.apply_goal_snapshot(_goal_snapshot_from_status_array(msg))

        def _on_costmap(self, msg: Costmap) -> None:
            self.costmap_msg = msg
            self._warned_missing_costmap = False

        def _lookup_robot_pose(self) -> Pose2D:
            transform = self.tf_buffer.lookup_transform(
                self.params.planning_frame,
                self.params.base_frame,
                Time(),
            )
            return Pose2D(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                yaw=_yaw_from_quaternion(transform.transform.rotation),
            )

        def _recovery_is_active(self) -> bool:
            if self.last_recovery_cmd_at is None:
                return False
            return (self.get_clock().now() - self.last_recovery_cmd_at) <= Duration(
                seconds=self.params.recovery_cmd_timeout
            )

        def _path_is_stale(self) -> bool:
            if self.last_path_received_at is None:
                return True
            return (self.get_clock().now() - self.last_path_received_at) > Duration(
                seconds=self.params.plan_stale_timeout
            )

        def _path_points(self) -> List[PathPoint2D]:
            return [
                PathPoint2D(
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    yaw=_yaw_from_quaternion(pose.pose.orientation),
                )
                for pose in self.path_msg.poses
            ]

        def _costmap_model(self) -> GridCostmap2D:
            metadata = self.costmap_msg.metadata
            return GridCostmap2D(
                origin_x=metadata.origin.position.x,
                origin_y=metadata.origin.position.y,
                resolution=metadata.resolution,
                width=metadata.size_x,
                height=metadata.size_y,
                data=tuple(self.costmap_msg.data),
            )

        def _make_costmap_validator(self) -> Callable[[PathPoint2D, PathPoint2D], bool]:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_msg.header.frame_id,
                self.params.planning_frame,
                Time(),
            )
            costmap = self._costmap_model()

            def validate(start: PathPoint2D, end: PathPoint2D) -> bool:
                return segment_is_costmap_safe(
                    costmap,
                    _transform_path_point(transform, start),
                    _transform_path_point(transform, end),
                    self.params.costmap_sample_step,
                    self.params.costmap_unsafe_cost,
                )

            return validate

        def _publish_twist(self, publisher: Any, command: Command2D) -> None:
            msg = Twist()
            msg.linear.x = command.linear_x
            msg.angular.z = command.angular_z
            publisher.publish(msg)

        def _publish_command(self, command: Command2D) -> None:
            routed = route_executor_command(self.params.executor_mode, command)
            if routed.active is not None:
                self._publish_twist(self.cmd_pub, routed.active)
            if routed.shadow is not None:
                self._publish_twist(self.shadow_cmd_pub, routed.shadow)

        def _publish_recovery_command(self) -> None:
            routed = route_executor_command(
                self.params.executor_mode,
                Command2D(
                    self.last_recovery_cmd.linear.x,
                    self.last_recovery_cmd.angular.z,
                ),
            )
            if routed.active is not None:
                self._publish_twist(self.cmd_pub, routed.active)
            if routed.shadow is not None:
                self._publish_twist(self.shadow_cmd_pub, routed.shadow)

        def _publish_segment_debug(self, segment: Any, progress: Any) -> None:
            if segment is None:
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                self.segment_pub.publish(delete_marker)

                empty_path = Path()
                self.segment_path_pub.publish(empty_path)
                return

            marker = Marker()
            marker.header.frame_id = self.params.planning_frame
            marker.ns = "go2w_motion_executor"
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.points = [
                Point(x=segment.start.x, y=segment.start.y, z=0.0),
                Point(x=segment.end.x, y=segment.end.y, z=0.0),
            ]
            self.segment_pub.publish(marker)

            segment_path = Path()
            segment_path.header.frame_id = self.params.planning_frame
            for point in (segment.start, segment.end):
                pose = PoseStamped()
                pose.header.frame_id = self.params.planning_frame
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.orientation.w = 1.0
                segment_path.poses.append(pose)
            self.segment_path_pub.publish(segment_path)

            self.along_track_pub.publish(Float64(data=progress.longitudinal_progress))
            self.cross_track_pub.publish(Float64(data=progress.cross_track_error))

        def _publish_state_debug(self) -> None:
            self.state_pub.publish(String(data=self.state.phase.value))
            self.goal_status_pub.publish(String(data=self.state.goal_lifecycle.value))
            self.stop_reason_pub.publish(String(data=self.state.stop_reason.value))

        def _publish_zero_and_debug(self) -> None:
            self._publish_command(self.shaper.step(Command2D(0.0, 0.0), self.control_period))
            self._publish_state_debug()

        def _target_rotate_command(self, yaw_error: float) -> Command2D:
            if abs(yaw_error) <= 1e-6:
                return Command2D(0.0, 0.0)
            magnitude = min(
                self.params.max_rotate_speed,
                abs(self.params.k_yaw * yaw_error),
            )
            magnitude = max(magnitude, self.params.min_rotate_speed)
            return Command2D(0.0, math.copysign(magnitude, yaw_error))

        def _target_drive_command(self, remaining_along_track: float) -> Command2D:
            if remaining_along_track <= self.params.slowdown_distance:
                scale = max(
                    0.25,
                    remaining_along_track / max(self.params.slowdown_distance, 1e-6),
                )
                return Command2D(self.params.drive_speed * scale, 0.0)
            return Command2D(self.params.drive_speed, 0.0)

        def _on_timer(self) -> None:
            measured = MeasuredMotion(
                linear_speed=self.odom_msg.twist.twist.linear.x if self.odom_msg else 0.0,
                angular_speed=self.odom_msg.twist.twist.angular.z if self.odom_msg else 0.0,
            )
            self.state.update_stopped(measured, self.control_period)

            if self._recovery_is_active():
                self.active_segment = None
                self.state.enter_recovery_passthrough()
                self._publish_segment_debug(None, None)
                self._publish_recovery_command()
                self._publish_state_debug()
                return

            if self.state.phase == MotionPhase.RECOVERY_PASSTHROUGH:
                self.active_segment = None
                self.state.exit_recovery_passthrough()

            try:
                robot_pose = self._lookup_robot_pose()
                self.last_robot_pose = robot_pose
            except TransformException:
                if self.state.phase != MotionPhase.STOPPING:
                    next_phase = (
                        MotionPhase.WAIT_FOR_PATH
                        if self.state.goal_lifecycle == GoalLifecycle.ACTIVE
                        else MotionPhase.IDLE
                    )
                    self.state.request_stop_for_phase(
                        next_phase, StopReason.TF_UNAVAILABLE
                    )
                self.active_segment = None
                self._publish_segment_debug(None, None)
                self._publish_zero_and_debug()
                return

            if self.state.phase == MotionPhase.STOPPING:
                self._publish_zero_and_debug()
                return

            if self.state.goal_lifecycle != GoalLifecycle.ACTIVE:
                self.drive_start_pose = None
                self.active_segment = None
                self._publish_segment_debug(None, None)
                self._publish_zero_and_debug()
                return

            if self.state.phase != MotionPhase.DRIVE and self.pending_path_msg is not None:
                self.path_msg = self.pending_path_msg
                self.last_path_received_at = self.get_clock().now()
                self.pending_path_msg = None

            if self.path_msg is None or len(self.path_msg.poses) < 2:
                self.state.phase = MotionPhase.WAIT_FOR_PATH
                self.drive_start_pose = None
                self.active_segment = None
                self._publish_segment_debug(None, None)
                self._publish_zero_and_debug()
                return

            if self._path_is_stale() and should_stop_for_stale_path(self.state.phase):
                self.active_segment = None
                self.state.handle_path_stale()
                self._publish_segment_debug(None, None)
                self._publish_zero_and_debug()
                return

            path_points = self._path_points()
            final_goal = path_points[-1]
            final_yaw_error = _shortest_angular_distance(robot_pose.yaw, final_goal.yaw)

            working_params = self.params
            line_is_safe = None
            if self.costmap_msg is None:
                if not self._warned_missing_costmap:
                    self.get_logger().warn(
                        "Local costmap unavailable; using degraded geometric-only segment validation"
                    )
                    self._warned_missing_costmap = True
                working_params = replace(
                    self.params,
                    max_segment_length=min(
                        self.params.max_segment_length,
                        self.params.degraded_max_segment_length,
                    ),
                )
            else:
                try:
                    line_is_safe = self._make_costmap_validator()
                except TransformException:
                    if self.state.phase != MotionPhase.STOPPING:
                        self.state.request_stop_for_phase(
                            MotionPhase.WAIT_FOR_PATH,
                            StopReason.TF_UNAVAILABLE,
                        )
                    self.active_segment = None
                    self._publish_segment_debug(None, None)
                    self._publish_zero_and_debug()
                    return

            try:
                segment = extract_straight_segment(
                    path_points,
                    robot_pose,
                    working_params,
                    line_is_safe=line_is_safe,
                )
            except ValueError:
                if self.state.phase != MotionPhase.STOPPING:
                    self.state.request_stop_for_phase(
                        MotionPhase.WAIT_FOR_PATH,
                        StopReason.PATH_REPLAN,
                    )
                self.active_segment = None
                self._publish_segment_debug(None, None)
                self._publish_zero_and_debug()
                return

            self.active_segment = segment
            progress = segment_progress(robot_pose, segment)
            yaw_error = _shortest_angular_distance(robot_pose.yaw, segment.yaw)

            self.yaw_error_pub.publish(Float64(data=yaw_error))
            self._publish_segment_debug(segment, progress)

            if self.state.phase in (MotionPhase.IDLE, MotionPhase.WAIT_FOR_PATH):
                if abs(yaw_error) <= self.params.rotate_exit_threshold:
                    self.state.phase = MotionPhase.DRIVE
                else:
                    self.state.phase = MotionPhase.ROTATE

            target = Command2D(0.0, 0.0)
            if self.state.phase == MotionPhase.ROTATE:
                self.drive_start_pose = None
                if abs(yaw_error) <= self.params.rotate_exit_threshold:
                    self.state.phase = MotionPhase.DRIVE
                else:
                    target = self._target_rotate_command(yaw_error)
            elif self.state.phase == MotionPhase.DRIVE:
                if self.drive_start_pose is None:
                    self.drive_start_pose = robot_pose
                driven_distance = math.hypot(
                    robot_pose.x - self.drive_start_pose.x,
                    robot_pose.y - self.drive_start_pose.y,
                )
                target = self._target_drive_command(
                    max(progress.remaining_along_track, 0.0)
                )
                if should_interrupt_drive_for_heading(
                    driven_distance=driven_distance,
                    yaw_error=yaw_error,
                    params=self.params,
                ):
                    self.state.request_stop_for_phase(
                        MotionPhase.ROTATE,
                        StopReason.HEADING_MISALIGNED,
                    )
                    target = Command2D(0.0, 0.0)
                elif (
                    progress.cross_track_error > self.params.max_cross_track_error
                    and can_leave_drive(
                        driven_distance,
                        safety_required=True,
                        params=self.params,
                    )
                ):
                    self.state.request_stop_for_phase(
                        MotionPhase.ROTATE,
                        StopReason.CROSS_TRACK_EXCEEDED,
                    )
                    target = Command2D(0.0, 0.0)
                elif should_accept_segment_completion(
                    driven_distance=driven_distance,
                    segment_complete=is_segment_complete(
                        progress,
                        self.params,
                        segment,
                    ),
                    params=self.params,
                ):
                    next_phase = (
                        MotionPhase.FINAL_ROTATE
                        if segment.end_index == len(path_points) - 1
                        else MotionPhase.ROTATE
                    )
                    self.state.request_stop_for_phase(
                        next_phase,
                        StopReason.SEGMENT_COMPLETE,
                    )
                    target = Command2D(0.0, 0.0)
            elif self.state.phase == MotionPhase.FINAL_ROTATE:
                self.drive_start_pose = None
                if abs(final_yaw_error) > self.params.shared_final_yaw_tolerance:
                    target = self._target_rotate_command(final_yaw_error)

            self._publish_command(self.shaper.step(target, self.control_period))
            self._publish_state_debug()

else:

    class Go2WMotionExecutorNode:  # pragma: no cover - used only without ROS.
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            _ensure_ros_imports()


def main(args: Optional[List[str]] = None) -> None:
    _ensure_ros_imports()
    rclpy.init(args=args)
    node = None
    try:
        node = Go2WMotionExecutorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


__all__ = [
    "DEFAULT_NODE_PARAMETERS",
    "Go2WMotionExecutorNode",
    "main",
    "merged_parameter_defaults",
]
