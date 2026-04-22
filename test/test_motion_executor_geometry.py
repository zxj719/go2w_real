import math
import pytest

from go2w_real.motion_executor_geometry import (
    GridCostmap2D,
    PathPoint2D,
    Pose2D,
    extract_straight_segment,
    is_segment_complete,
    segment_is_costmap_safe,
    segment_progress,
)
from go2w_real.motion_executor_types import MotionExecutorParams


def test_extract_straight_segment_stops_before_heading_break():
    params = MotionExecutorParams(segment_heading_change_limit=math.radians(12.0))
    robot = Pose2D(0.0, 0.0, 0.0)
    path = [
        PathPoint2D(0.0, 0.0, 0.0),
        PathPoint2D(0.5, 0.0, 0.0),
        PathPoint2D(1.0, 0.0, 0.0),
        PathPoint2D(1.0, 0.6, math.pi / 2.0),
    ]

    segment = extract_straight_segment(path, robot, params)

    assert segment.start_index == 0
    assert segment.end_index == 2
    assert math.isclose(segment.end.x, 1.0)
    assert math.isclose(segment.end.y, 0.0)


def test_segment_completion_uses_projection_margin_after_overshoot():
    params = MotionExecutorParams(
        segment_arrival_distance=0.12,
        max_cross_track_error=0.18,
        segment_pass_projection_margin=0.05,
    )
    path = [
        PathPoint2D(0.0, 0.0, 0.0),
        PathPoint2D(1.0, 0.0, 0.0),
    ]
    segment = extract_straight_segment(path, Pose2D(0.0, 0.0, 0.0), params)
    progress = segment_progress(Pose2D(1.08, 0.16, 0.0), segment)

    assert progress.longitudinal_progress > segment.length
    assert is_segment_complete(progress, params, segment)


def test_costmap_validation_truncates_segment_before_unsafe_cell():
    params = MotionExecutorParams(costmap_sample_step=0.05, costmap_unsafe_cost=200)
    data = [0] * 200
    data[(5 * 20) + 6] = 254
    costmap = GridCostmap2D(
        origin_x=0.0,
        origin_y=-0.5,
        resolution=0.1,
        width=20,
        height=10,
        data=tuple(data),
    )
    path = [
        PathPoint2D(0.0, 0.0, 0.0),
        PathPoint2D(0.5, 0.0, 0.0),
        PathPoint2D(1.0, 0.0, 0.0),
    ]

    validator = lambda start, end: segment_is_costmap_safe(
        costmap,
        start,
        end,
        params.costmap_sample_step,
        params.costmap_unsafe_cost,
    )
    segment = extract_straight_segment(path, Pose2D(0.0, 0.0, 0.0), params, line_is_safe=validator)

    assert segment.end_index == 1


def test_extract_straight_segment_rejects_first_edge_longer_than_max():
    params = MotionExecutorParams(max_segment_length=0.3)
    path = [
        PathPoint2D(0.0, 0.0, 0.0),
        PathPoint2D(0.5, 0.0, 0.0),
        PathPoint2D(0.7, 0.0, 0.0),
    ]

    with pytest.raises(ValueError):
        extract_straight_segment(path, Pose2D(0.0, 0.0, 0.0), params)


def test_extract_straight_segment_rejects_first_edge_when_unsafe():
    params = MotionExecutorParams()
    path = [
        PathPoint2D(0.0, 0.0, 0.0),
        PathPoint2D(0.2, 0.0, 0.0),
        PathPoint2D(0.4, 0.0, 0.0),
    ]

    with pytest.raises(ValueError):
        extract_straight_segment(
            path,
            Pose2D(0.0, 0.0, 0.0),
            params,
            line_is_safe=lambda _start, _end: False,
        )


def test_costmap_validation_marks_negative_off_map_sampling_as_unsafe():
    costmap = GridCostmap2D(
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
        width=5,
        height=5,
        data=tuple([0] * 25),
    )

    is_safe = segment_is_costmap_safe(
        costmap,
        PathPoint2D(-0.01, 1.0, 0.0),
        PathPoint2D(1.0, 1.0, 0.0),
        sample_step=0.1,
        unsafe_cost=200,
    )

    assert not is_safe
