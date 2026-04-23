from dataclasses import dataclass
from enum import Enum


class MotionPhase(Enum):
    IDLE = "IDLE"
    WAIT_FOR_PATH = "WAIT_FOR_PATH"
    ROTATE = "ROTATE"
    DRIVE = "DRIVE"
    STOPPING = "STOPPING"
    FINAL_ROTATE = "FINAL_ROTATE"
    RECOVERY_PASSTHROUGH = "RECOVERY_PASSTHROUGH"


class GoalLifecycle(Enum):
    NONE = "NONE"
    ACTIVE = "ACTIVE"
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    ABORTED = "ABORTED"
    PREEMPTED = "PREEMPTED"


class ExecutorMode(Enum):
    SHADOW = "shadow"
    ACTIVE = "active"


class StopReason(Enum):
    NONE = "NONE"
    SEGMENT_COMPLETE = "SEGMENT_COMPLETE"
    GOAL_TERMINAL = "GOAL_TERMINAL"
    GOAL_PREEMPTED = "GOAL_PREEMPTED"
    PATH_STALE = "PATH_STALE"
    PATH_REPLAN = "PATH_REPLAN"
    HEADING_MISALIGNED = "HEADING_MISALIGNED"
    CROSS_TRACK_EXCEEDED = "CROSS_TRACK_EXCEEDED"
    TF_UNAVAILABLE = "TF_UNAVAILABLE"


@dataclass(frozen=True)
class MotionExecutorParams:
    executor_mode: str = "shadow"
    shadow_cmd_vel_topic: str = "/go2w_motion_executor/shadow_cmd_vel"
    navigation_status_topic: str = "/navigate_to_pose/_action/status"
    planning_frame: str = "map"
    base_frame: str = "base"
    shared_final_yaw_tolerance: float = 3.14
    max_segment_length: float = 1.0
    degraded_max_segment_length: float = 0.5
    segment_heading_change_limit: float = 0.2094395
    segment_lateral_deviation_limit: float = 0.10
    segment_pass_projection_margin: float = 0.05
    rotate_enter_threshold: float = 0.2094395
    rotate_exit_threshold: float = 0.0698132
    k_yaw: float = 1.2
    min_rotate_speed: float = 0.18
    max_rotate_speed: float = 0.75
    drive_speed: float = 0.22
    slowdown_distance: float = 0.50
    segment_arrival_distance: float = 0.12
    min_drive_commit_distance: float = 0.30
    max_cross_track_error: float = 0.18
    max_forward_accel: float = 0.20
    max_forward_decel: float = 0.30
    max_yaw_accel: float = 0.40
    max_yaw_decel: float = 0.60
    stopped_linear_velocity_threshold: float = 0.03
    stopped_angular_velocity_threshold: float = 0.05
    stopped_hold_time: float = 0.20
    plan_switch_heading_tolerance: float = 0.1396263
    plan_switch_lateral_tolerance: float = 0.15
    plan_stale_timeout: float = 0.8
    costmap_sample_step: float = 0.05
    costmap_unsafe_cost: int = 200
    recovery_cmd_timeout: float = 0.25
