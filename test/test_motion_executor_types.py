from dataclasses import asdict
from dataclasses import fields

from go2w_real.motion_executor_node import (
    DEFAULT_NODE_PARAMETERS,
    merged_parameter_defaults,
)
from go2w_real.motion_executor_types import (
    ExecutorMode,
    GoalLifecycle,
    MotionExecutorParams,
    MotionPhase,
    StopReason,
)


def test_motion_executor_shared_defaults_match_design_contract():
    assert [(mode.name, mode.value) for mode in ExecutorMode] == [
        ("SHADOW", "shadow"),
        ("ACTIVE", "active"),
    ]
    assert [(phase.name, phase.value) for phase in MotionPhase] == [
        ("IDLE", "IDLE"),
        ("WAIT_FOR_PATH", "WAIT_FOR_PATH"),
        ("ROTATE", "ROTATE"),
        ("DRIVE", "DRIVE"),
        ("STOPPING", "STOPPING"),
        ("FINAL_ROTATE", "FINAL_ROTATE"),
        ("RECOVERY_PASSTHROUGH", "RECOVERY_PASSTHROUGH"),
    ]
    assert [(state.name, state.value) for state in GoalLifecycle] == [
        ("NONE", "NONE"),
        ("ACTIVE", "ACTIVE"),
        ("SUCCEEDED", "SUCCEEDED"),
        ("CANCELED", "CANCELED"),
        ("ABORTED", "ABORTED"),
        ("PREEMPTED", "PREEMPTED"),
    ]
    assert [(reason.name, reason.value) for reason in StopReason] == [
        ("NONE", "NONE"),
        ("SEGMENT_COMPLETE", "SEGMENT_COMPLETE"),
        ("GOAL_TERMINAL", "GOAL_TERMINAL"),
        ("GOAL_PREEMPTED", "GOAL_PREEMPTED"),
        ("PATH_STALE", "PATH_STALE"),
        ("PATH_REPLAN", "PATH_REPLAN"),
        ("HEADING_MISALIGNED", "HEADING_MISALIGNED"),
        ("CROSS_TRACK_EXCEEDED", "CROSS_TRACK_EXCEEDED"),
        ("TF_UNAVAILABLE", "TF_UNAVAILABLE"),
    ]
    assert asdict(MotionExecutorParams()) == {
        "executor_mode": "shadow",
        "shadow_cmd_vel_topic": "/go2w_motion_executor/shadow_cmd_vel",
        "navigation_status_topic": "/navigate_to_pose/_action/status",
        "planning_frame": "map",
        "base_frame": "base",
        "shared_final_yaw_tolerance": 3.14,
        "max_segment_length": 1.0,
        "degraded_max_segment_length": 0.5,
        "segment_heading_change_limit": 0.2094395,
        "segment_lateral_deviation_limit": 0.10,
        "segment_pass_projection_margin": 0.05,
        "rotate_enter_threshold": 0.2094395,
        "rotate_exit_threshold": 0.0698132,
        "k_yaw": 1.2,
        "min_rotate_speed": 0.18,
        "max_rotate_speed": 0.45,
        "drive_speed": 0.22,
        "slowdown_distance": 0.50,
        "segment_arrival_distance": 0.12,
        "min_drive_commit_distance": 0.30,
        "max_cross_track_error": 0.18,
        "max_forward_accel": 0.20,
        "max_forward_decel": 0.30,
        "max_yaw_accel": 0.40,
        "max_yaw_decel": 0.60,
        "stopped_linear_velocity_threshold": 0.03,
        "stopped_angular_velocity_threshold": 0.05,
        "stopped_hold_time": 0.20,
        "plan_switch_heading_tolerance": 0.1396263,
        "plan_switch_lateral_tolerance": 0.15,
        "plan_stale_timeout": 0.8,
        "costmap_sample_step": 0.05,
        "costmap_unsafe_cost": 200,
        "recovery_cmd_timeout": 0.25,
    }


def test_motion_executor_node_defaults_match_spec_topics():
    assert DEFAULT_NODE_PARAMETERS["executor_mode"] == "shadow"
    assert (
        DEFAULT_NODE_PARAMETERS["shadow_cmd_vel_topic"]
        == "/go2w_motion_executor/shadow_cmd_vel"
    )
    assert (
        DEFAULT_NODE_PARAMETERS["navigation_status_topic"]
        == "/navigate_to_pose/_action/status"
    )
    assert DEFAULT_NODE_PARAMETERS["path_topic"] == "/plan"
    assert DEFAULT_NODE_PARAMETERS["nav2_cmd_vel_topic"] == "/nav2_cmd_vel_raw"
    assert (
        DEFAULT_NODE_PARAMETERS["nav2_recovery_cmd_vel_topic"]
        == "/nav2_recovery_cmd_vel"
    )
    assert DEFAULT_NODE_PARAMETERS["local_costmap_topic"] == "/local_costmap/costmap"


def test_motion_executor_node_parameter_defaults_merge_without_duplicate_names():
    merged = merged_parameter_defaults()
    motion_param_names = {field.name for field in fields(MotionExecutorParams)}
    expected_names = set(DEFAULT_NODE_PARAMETERS) | motion_param_names

    assert set(merged) == expected_names
    assert len(merged) == len(expected_names)
    assert merged["navigation_status_topic"] == "/navigate_to_pose/_action/status"
    assert merged["path_topic"] == "/plan"
    assert merged["planning_frame"] == "map"
