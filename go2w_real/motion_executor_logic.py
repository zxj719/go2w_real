from dataclasses import dataclass
import math
from typing import Optional

from .motion_executor_types import (
    ExecutorMode,
    GoalLifecycle,
    MotionExecutorParams,
    MotionPhase,
    StopReason,
)


@dataclass(frozen=True)
class Command2D:
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class MeasuredMotion:
    linear_speed: float
    angular_speed: float


@dataclass(frozen=True)
class GoalStatusSnapshot:
    goal_id: Optional[str]
    lifecycle: GoalLifecycle


@dataclass(frozen=True)
class RoutedCommand:
    active: Optional[Command2D]
    shadow: Optional[Command2D]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def route_executor_command(
    executor_mode: str,
    command: Command2D,
) -> RoutedCommand:
    if executor_mode == ExecutorMode.ACTIVE.value:
        return RoutedCommand(active=command, shadow=None)
    return RoutedCommand(active=None, shadow=command)


def should_switch_path(
    current_heading: float,
    new_heading: float,
    lateral_offset: float,
    covers_continuation: bool,
    in_drive: bool,
    params: MotionExecutorParams,
) -> bool:
    if not in_drive:
        return True
    if (
        abs(_normalize_angle(new_heading - current_heading))
        > params.plan_switch_heading_tolerance
    ):
        return True
    if lateral_offset > params.plan_switch_lateral_tolerance:
        return True
    if not covers_continuation:
        return True
    return False


def can_leave_drive(
    driven_distance: float,
    safety_required: bool,
    params: MotionExecutorParams,
) -> bool:
    return safety_required or driven_distance >= params.min_drive_commit_distance


def should_accept_segment_completion(
    driven_distance: float,
    segment_complete: bool,
    params: MotionExecutorParams,
) -> bool:
    if not segment_complete:
        return False
    return can_leave_drive(
        driven_distance=driven_distance,
        safety_required=False,
        params=params,
    )


def should_interrupt_drive_for_heading(
    driven_distance: float,
    yaw_error: float,
    params: MotionExecutorParams,
) -> bool:
    return (
        can_leave_drive(
            driven_distance=driven_distance,
            safety_required=False,
            params=params,
        )
        and abs(_normalize_angle(yaw_error)) > params.rotate_enter_threshold
    )


def should_stop_for_stale_path(phase: MotionPhase) -> bool:
    return phase in (
        MotionPhase.WAIT_FOR_PATH,
        MotionPhase.ROTATE,
        MotionPhase.DRIVE,
        MotionPhase.FINAL_ROTATE,
    )


class CommandShaper:
    def __init__(self, params: MotionExecutorParams):
        self._params = params
        self._current = Command2D(0.0, 0.0)

    def _ramp(
        self,
        current: float,
        target: float,
        accel_limit: float,
        decel_limit: float,
        dt: float,
    ) -> float:
        limit = accel_limit if abs(target) > abs(current) else decel_limit
        delta = target - current
        max_delta = limit * dt
        if delta > max_delta:
            delta = max_delta
        if delta < -max_delta:
            delta = -max_delta
        value = current + delta
        return 0.0 if abs(value) < 1e-6 else value

    def step(self, target: Command2D, dt: float) -> Command2D:
        if abs(target.linear_x) > 1e-6 and abs(self._current.angular_z) > 1e-6:
            target = Command2D(0.0, 0.0)
        if abs(target.angular_z) > 1e-6 and abs(self._current.linear_x) > 1e-6:
            target = Command2D(0.0, 0.0)

        next_linear = self._ramp(
            self._current.linear_x,
            target.linear_x,
            self._params.max_forward_accel,
            self._params.max_forward_decel,
            dt,
        )
        next_angular = self._ramp(
            self._current.angular_z,
            target.angular_z,
            self._params.max_yaw_accel,
            self._params.max_yaw_decel,
            dt,
        )
        self._current = Command2D(next_linear, next_angular)
        return self._current


class MotionExecutorState:
    def __init__(self, params: MotionExecutorParams):
        self.params = params
        self.phase = MotionPhase.IDLE
        self.goal_lifecycle = GoalLifecycle.NONE
        self.active_goal_id: Optional[str] = None
        self.pending_goal_id: Optional[str] = None
        self.next_phase_after_stop = MotionPhase.IDLE
        self.stop_reason = StopReason.NONE
        self._stopped_time = 0.0

    def apply_goal_snapshot(self, snapshot: GoalStatusSnapshot) -> None:
        if snapshot.lifecycle == GoalLifecycle.ACTIVE:
            if self.active_goal_id is None:
                self.active_goal_id = snapshot.goal_id
                self.goal_lifecycle = GoalLifecycle.ACTIVE
                if self.phase == MotionPhase.IDLE:
                    self.phase = MotionPhase.WAIT_FOR_PATH
                return

            if snapshot.goal_id != self.active_goal_id:
                self.pending_goal_id = snapshot.goal_id
                self.goal_lifecycle = GoalLifecycle.PREEMPTED
                self.request_stop_for_phase(
                    MotionPhase.WAIT_FOR_PATH, StopReason.GOAL_PREEMPTED
                )
                return

            self.goal_lifecycle = GoalLifecycle.ACTIVE
            return

        if self.pending_goal_id is not None and snapshot.goal_id not in (
            None,
            self.pending_goal_id,
        ):
            return

        if (
            self.pending_goal_id is None
            and self.active_goal_id is not None
            and snapshot.goal_id not in (None, self.active_goal_id)
        ):
            return

        self.goal_lifecycle = snapshot.lifecycle
        if snapshot.lifecycle in (
            GoalLifecycle.SUCCEEDED,
            GoalLifecycle.CANCELED,
            GoalLifecycle.ABORTED,
        ):
            self.request_stop_for_phase(MotionPhase.IDLE, StopReason.GOAL_TERMINAL)

    def request_stop_for_phase(self, next_phase: MotionPhase, reason: StopReason) -> None:
        self.phase = MotionPhase.STOPPING
        self.next_phase_after_stop = next_phase
        self.stop_reason = reason
        self._stopped_time = 0.0

    def handle_path_stale(self) -> None:
        next_phase = (
            MotionPhase.WAIT_FOR_PATH
            if self.goal_lifecycle == GoalLifecycle.ACTIVE
            else MotionPhase.IDLE
        )
        self.request_stop_for_phase(next_phase, StopReason.PATH_STALE)

    def enter_recovery_passthrough(self) -> None:
        self.phase = MotionPhase.RECOVERY_PASSTHROUGH
        self.stop_reason = StopReason.NONE

    def exit_recovery_passthrough(self) -> None:
        self.phase = (
            MotionPhase.WAIT_FOR_PATH
            if self.goal_lifecycle == GoalLifecycle.ACTIVE
            else MotionPhase.IDLE
        )

    def update_stopped(self, measured: MeasuredMotion, dt: float) -> None:
        if self.phase != MotionPhase.STOPPING:
            return

        is_stopped = (
            abs(measured.linear_speed) <= self.params.stopped_linear_velocity_threshold
            and abs(measured.angular_speed) <= self.params.stopped_angular_velocity_threshold
        )
        self._stopped_time = self._stopped_time + dt if is_stopped else 0.0

        if self._stopped_time < self.params.stopped_hold_time:
            return

        self.phase = self.next_phase_after_stop
        self._stopped_time = 0.0

        if self.phase == MotionPhase.WAIT_FOR_PATH and self.pending_goal_id is not None:
            self.active_goal_id = self.pending_goal_id
            self.pending_goal_id = None
            self.goal_lifecycle = GoalLifecycle.ACTIVE
        elif self.phase == MotionPhase.IDLE:
            self.active_goal_id = None
            self.pending_goal_id = None

        self.stop_reason = StopReason.NONE
