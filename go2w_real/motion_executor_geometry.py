from dataclasses import dataclass
import math
from typing import Callable, Optional, Sequence, Tuple

from .motion_executor_types import MotionExecutorParams


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PathPoint2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StraightSegment:
    start: PathPoint2D
    end: PathPoint2D
    start_index: int
    end_index: int
    length: float
    yaw: float


@dataclass(frozen=True)
class SegmentProgress:
    longitudinal_progress: float
    remaining_along_track: float
    cross_track_error: float


@dataclass(frozen=True)
class GridCostmap2D:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    data: Tuple[int, ...]


def _distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _heading(a: PathPoint2D, b: PathPoint2D) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def _line_cross_track(point_x: float, point_y: float, start: PathPoint2D, end: PathPoint2D) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    denom = math.hypot(dx, dy)
    if denom == 0.0:
        return math.hypot(point_x - start.x, point_y - start.y)
    return abs((dy * point_x) - (dx * point_y) + (end.x * start.y) - (end.y * start.x)) / denom


def _closest_usable_index(path: Sequence[PathPoint2D], robot_pose: Pose2D) -> int:
    fallback = min(
        range(len(path) - 1),
        key=lambda idx: _distance(robot_pose.x, robot_pose.y, path[idx].x, path[idx].y),
    )
    best_index = fallback
    best_distance = _distance(robot_pose.x, robot_pose.y, path[fallback].x, path[fallback].y)

    for idx in range(len(path) - 1):
        start = path[idx]
        end = path[idx + 1]
        seg_dx = end.x - start.x
        seg_dy = end.y - start.y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len == 0.0:
            continue
        along_track = ((robot_pose.x - start.x) * seg_dx + (robot_pose.y - start.y) * seg_dy) / seg_len
        if along_track < 0.0:
            continue
        distance = _distance(robot_pose.x, robot_pose.y, start.x, start.y)
        if distance < best_distance:
            best_distance = distance
            best_index = idx

    return best_index


def segment_progress(robot_pose: Pose2D, segment: StraightSegment) -> SegmentProgress:
    dx = segment.end.x - segment.start.x
    dy = segment.end.y - segment.start.y
    segment_length = max(segment.length, 1e-9)
    ux = dx / segment_length
    uy = dy / segment_length
    rel_x = robot_pose.x - segment.start.x
    rel_y = robot_pose.y - segment.start.y
    longitudinal = rel_x * ux + rel_y * uy
    remaining = segment.length - longitudinal
    cross_track = _line_cross_track(robot_pose.x, robot_pose.y, segment.start, segment.end)
    return SegmentProgress(longitudinal, remaining, cross_track)


def is_segment_complete(
    progress: SegmentProgress,
    params: MotionExecutorParams,
    segment: StraightSegment,
) -> bool:
    within_endpoint = (
        progress.remaining_along_track <= params.segment_arrival_distance
        and progress.cross_track_error <= params.max_cross_track_error
    )
    overshot = progress.longitudinal_progress >= (segment.length + params.segment_pass_projection_margin)
    return within_endpoint or overshot


def segment_is_costmap_safe(
    costmap: GridCostmap2D,
    start: PathPoint2D,
    end: PathPoint2D,
    sample_step: float,
    unsafe_cost: int,
) -> bool:
    length = _distance(start.x, start.y, end.x, end.y)
    steps = max(1, int(math.ceil(length / max(sample_step, 1e-6))))

    for index in range(steps + 1):
        ratio = index / steps
        world_x = start.x + ((end.x - start.x) * ratio)
        world_y = start.y + ((end.y - start.y) * ratio)
        map_x = math.floor((world_x - costmap.origin_x) / costmap.resolution)
        map_y = math.floor((world_y - costmap.origin_y) / costmap.resolution)

        if map_x < 0 or map_x >= costmap.width or map_y < 0 or map_y >= costmap.height:
            return False

        cost = costmap.data[(map_y * costmap.width) + map_x]
        if cost >= unsafe_cost:
            return False

    return True


def extract_straight_segment(
    path: Sequence[PathPoint2D],
    robot_pose: Pose2D,
    params: MotionExecutorParams,
    line_is_safe: Optional[Callable[[PathPoint2D, PathPoint2D], bool]] = None,
) -> StraightSegment:
    if len(path) < 2:
        raise ValueError("path must contain at least two points")

    start_index = _closest_usable_index(path, robot_pose)
    start = path[start_index]
    end_index: Optional[int] = None
    end: Optional[PathPoint2D] = None

    previous_heading = 0.0
    cumulative_heading_change = 0.0

    for candidate_index in range(start_index + 1, len(path)):
        candidate = path[candidate_index]
        candidate_length = _distance(start.x, start.y, candidate.x, candidate.y)
        if candidate_length > params.max_segment_length:
            break

        if candidate_index == start_index + 1:
            previous_heading = _heading(start, candidate)
        else:
            edge_heading = _heading(path[candidate_index - 1], candidate)
            cumulative_heading_change += abs(_normalize_angle(edge_heading - previous_heading))
            previous_heading = edge_heading
            if cumulative_heading_change > params.segment_heading_change_limit:
                break

        max_lateral_deviation = max(
            _line_cross_track(path[idx].x, path[idx].y, start, candidate)
            for idx in range(start_index + 1, candidate_index + 1)
        )
        if max_lateral_deviation > params.segment_lateral_deviation_limit:
            break

        if line_is_safe is not None and not line_is_safe(start, candidate):
            break

        end_index = candidate_index
        end = candidate

    if end_index is None or end is None:
        raise ValueError("no valid straight segment from path constraints")

    return StraightSegment(
        start=start,
        end=end,
        start_index=start_index,
        end_index=end_index,
        length=_distance(start.x, start.y, end.x, end.y),
        yaw=_heading(start, end),
    )
