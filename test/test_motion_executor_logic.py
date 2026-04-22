from go2w_real.motion_executor_logic import (
    can_leave_drive,
    Command2D,
    CommandShaper,
    GoalStatusSnapshot,
    MeasuredMotion,
    MotionExecutorState,
    RoutedCommand,
    route_executor_command,
    should_accept_segment_completion,
    should_interrupt_drive_for_heading,
    should_stop_for_stale_path,
    should_switch_path,
)
from go2w_real.motion_executor_types import (
    ExecutorMode,
    GoalLifecycle,
    MotionExecutorParams,
    MotionPhase,
    StopReason,
)


def test_terminal_goal_transitions_from_drive_to_stopping_then_idle():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.phase = MotionPhase.DRIVE
    state.active_goal_id = "goal-1"
    state.goal_lifecycle = GoalLifecycle.ACTIVE

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.CANCELED))

    assert state.phase == MotionPhase.STOPPING
    assert state.stop_reason == StopReason.GOAL_TERMINAL

    state.update_stopped(MeasuredMotion(0.0, 0.0), dt=0.25)
    assert state.phase == MotionPhase.IDLE


def test_preempted_goal_transitions_to_wait_for_path_after_stop():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.ACTIVE))
    state.phase = MotionPhase.DRIVE

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-2", GoalLifecycle.ACTIVE))

    assert state.phase == MotionPhase.STOPPING
    assert state.goal_lifecycle == GoalLifecycle.PREEMPTED
    assert state.stop_reason == StopReason.GOAL_PREEMPTED

    state.update_stopped(MeasuredMotion(0.0, 0.0), dt=0.25)
    assert state.phase == MotionPhase.WAIT_FOR_PATH
    assert state.active_goal_id == "goal-2"


def test_terminal_update_for_old_goal_does_not_drop_pending_preempted_goal():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.ACTIVE))
    state.phase = MotionPhase.DRIVE

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-2", GoalLifecycle.ACTIVE))
    assert state.phase == MotionPhase.STOPPING
    assert state.next_phase_after_stop == MotionPhase.WAIT_FOR_PATH
    assert state.pending_goal_id == "goal-2"

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.CANCELED))

    assert state.phase == MotionPhase.STOPPING
    assert state.next_phase_after_stop == MotionPhase.WAIT_FOR_PATH

    state.update_stopped(MeasuredMotion(0.0, 0.0), dt=0.25)
    assert state.phase == MotionPhase.WAIT_FOR_PATH
    assert state.active_goal_id == "goal-2"
    assert state.goal_lifecycle == GoalLifecycle.ACTIVE


def test_delayed_terminal_update_for_old_goal_after_handoff_is_ignored():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.ACTIVE))
    state.phase = MotionPhase.DRIVE

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-2", GoalLifecycle.ACTIVE))
    state.update_stopped(MeasuredMotion(0.0, 0.0), dt=0.25)
    assert state.phase == MotionPhase.WAIT_FOR_PATH
    assert state.active_goal_id == "goal-2"
    assert state.goal_lifecycle == GoalLifecycle.ACTIVE

    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.CANCELED))

    assert state.phase == MotionPhase.WAIT_FOR_PATH
    assert state.active_goal_id == "goal-2"
    assert state.goal_lifecycle == GoalLifecycle.ACTIVE


def test_path_stale_keeps_active_goal_but_returns_to_wait_for_path():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.ACTIVE))
    state.phase = MotionPhase.DRIVE

    state.handle_path_stale()

    assert state.phase == MotionPhase.STOPPING
    assert state.next_phase_after_stop == MotionPhase.WAIT_FOR_PATH
    assert state.stop_reason == StopReason.PATH_STALE


def test_recovery_passthrough_resumes_wait_for_path_after_recovery():
    params = MotionExecutorParams()
    state = MotionExecutorState(params)
    state.apply_goal_snapshot(GoalStatusSnapshot("goal-1", GoalLifecycle.ACTIVE))
    state.phase = MotionPhase.DRIVE

    state.enter_recovery_passthrough()
    assert state.phase == MotionPhase.RECOVERY_PASSTHROUGH

    state.exit_recovery_passthrough()
    assert state.phase == MotionPhase.WAIT_FOR_PATH


def test_path_switch_logic_holds_small_replan_during_drive():
    params = MotionExecutorParams(
        plan_switch_heading_tolerance=0.1396263, plan_switch_lateral_tolerance=0.15
    )

    assert not should_switch_path(
        current_heading=0.0,
        new_heading=0.05,
        lateral_offset=0.04,
        covers_continuation=True,
        in_drive=True,
        params=params,
    )


def test_drive_commit_allows_safety_interrupt_but_blocks_minor_replan():
    params = MotionExecutorParams(min_drive_commit_distance=0.30)

    assert not can_leave_drive(
        driven_distance=0.10, safety_required=False, params=params
    )
    assert can_leave_drive(driven_distance=0.10, safety_required=True, params=params)


def test_segment_completion_is_blocked_until_drive_commit_is_met():
    params = MotionExecutorParams(min_drive_commit_distance=0.30)

    assert not should_accept_segment_completion(
        driven_distance=0.10,
        segment_complete=True,
        params=params,
    )
    assert should_accept_segment_completion(
        driven_distance=0.30,
        segment_complete=True,
        params=params,
    )
    assert not should_accept_segment_completion(
        driven_distance=0.50,
        segment_complete=False,
        params=params,
    )


def test_shadow_mode_routes_segmented_command_to_shadow_output_only():
    routed = route_executor_command(
        ExecutorMode.SHADOW.value,
        Command2D(0.22, 0.0),
    )

    assert routed == RoutedCommand(
        active=None,
        shadow=Command2D(0.22, 0.0),
    )


def test_active_mode_routes_segmented_command_to_cmd_vel_only():
    routed = route_executor_command(
        ExecutorMode.ACTIVE.value,
        Command2D(0.0, 0.30),
    )

    assert routed == RoutedCommand(
        active=Command2D(0.0, 0.30),
        shadow=None,
    )


def test_drive_heading_error_uses_rotate_enter_threshold_after_commit():
    params = MotionExecutorParams(
        rotate_enter_threshold=0.20,
        min_drive_commit_distance=0.30,
    )

    assert should_interrupt_drive_for_heading(
        driven_distance=0.31,
        yaw_error=0.25,
        params=params,
    )
    assert not should_interrupt_drive_for_heading(
        driven_distance=0.10,
        yaw_error=0.25,
        params=params,
    )
    assert not should_interrupt_drive_for_heading(
        driven_distance=0.31,
        yaw_error=0.05,
        params=params,
    )


def test_stale_path_forces_stop_in_all_active_motion_phases():
    assert should_stop_for_stale_path(MotionPhase.WAIT_FOR_PATH)
    assert should_stop_for_stale_path(MotionPhase.ROTATE)
    assert should_stop_for_stale_path(MotionPhase.DRIVE)
    assert should_stop_for_stale_path(MotionPhase.FINAL_ROTATE)
    assert not should_stop_for_stale_path(MotionPhase.STOPPING)
    assert not should_stop_for_stale_path(MotionPhase.IDLE)


def test_command_shaper_forces_zero_crossing_between_drive_and_rotate():
    params = MotionExecutorParams()
    shaper = CommandShaper(params)

    drive = shaper.step(Command2D(0.22, 0.0), dt=1.0)
    assert drive.linear_x > 0.0

    zero_cross = shaper.step(Command2D(0.0, 0.30), dt=0.1)
    assert zero_cross.angular_z == 0.0

    zeroed = shaper.step(Command2D(0.0, 0.0), dt=1.0)
    rotated = shaper.step(Command2D(0.0, 0.30), dt=1.0)
    assert zeroed.linear_x == 0.0
    assert rotated.angular_z > 0.0
