#include "go2w_real/frontier_goal_selector.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace go2w_real
{
namespace
{

double distance_xy(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

bool is_goal_admissible(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const std::pair<double, double> & world_xy,
  double goal_clearance_radius)
{
  const auto grid_cell = grid_map.world_to_cell(world_xy.first, world_xy.second);
  const auto global_cell = global_costmap.world_to_cell(world_xy.first, world_xy.second);
  if (!grid_cell || !global_cell) {
    return false;
  }

  if (!grid_map.is_goal_free(grid_cell->first, grid_cell->second) ||
    !global_costmap.is_goal_free(global_cell->first, global_cell->second))
  {
    return false;
  }

  if (goal_clearance_radius <= 0.0) {
    return true;
  }

  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(goal_clearance_radius / global_costmap.resolution)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = global_cell->first + dx;
      const int y = global_cell->second + dy;
      if (!global_costmap.in_bounds(x, y)) {
        return false;
      }

      const auto candidate_xy = global_costmap.cell_center_world(x, y);
      if (distance_xy(candidate_xy, world_xy) <= goal_clearance_radius &&
        !global_costmap.is_goal_free(x, y))
      {
        return false;
      }
    }
  }

  return true;
}

std::vector<std::pair<double, double>> ring_candidates(
  const GridMapView & global_costmap,
  const std::pair<double, double> & anchor_xy,
  double snap_radius)
{
  std::vector<std::pair<double, double>> candidates;
  const auto anchor_cell = global_costmap.world_to_cell(anchor_xy.first, anchor_xy.second);
  if (!anchor_cell) {
    return candidates;
  }

  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(snap_radius / global_costmap.resolution)));
  const double tolerance = 0.5 * global_costmap.resolution + 1e-9;

  for (int dy = -radius_cells - 1; dy <= radius_cells + 1; ++dy) {
    for (int dx = -radius_cells - 1; dx <= radius_cells + 1; ++dx) {
      const int x = anchor_cell->first + dx;
      const int y = anchor_cell->second + dy;
      if (!global_costmap.in_bounds(x, y)) {
        continue;
      }

      const auto world_xy = global_costmap.cell_center_world(x, y);
      if (std::abs(distance_xy(world_xy, anchor_xy) - snap_radius) <= tolerance) {
        candidates.push_back(world_xy);
      }
    }
  }

  return candidates;
}

}  // namespace

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double suppression_radius)
{
  std::vector<Frontier> kept;
  kept.reserve(frontiers.size());

  for (const auto & frontier : frontiers) {
    bool suppressed = false;
    for (const auto & kept_frontier : kept) {
      if (std::hypot(
          frontier.centroid_x - kept_frontier.centroid_x,
          frontier.centroid_y - kept_frontier.centroid_y) <= suppression_radius)
      {
        suppressed = true;
        break;
      }
    }

    if (!suppressed) {
      kept.push_back(frontier);
    }
  }

  return kept;
}

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const FrontierGoalSelectorConfig & config)
{
  if (!grid_map.valid() || !global_costmap.valid()) {
    return std::nullopt;
  }

  const std::pair<double, double> anchor_xy{frontier.centroid_x, frontier.centroid_y};
  if (is_goal_admissible(grid_map, global_costmap, anchor_xy, config.goal_clearance_radius)) {
    return FrontierTarget{frontier, anchor_xy, anchor_xy};
  }

  double last_radius = -1.0;
  for (const double radius : std::array<double, 2>{
         config.snap_radius, config.fallback_snap_radius})
  {
    if (radius <= 0.0 || std::abs(radius - last_radius) <= 1e-6) {
      continue;
    }
    last_radius = radius;

    auto candidates = ring_candidates(global_costmap, anchor_xy, radius);
    std::sort(
      candidates.begin(), candidates.end(),
      [&robot_xy](const auto & a, const auto & b) {
        return distance_xy(a, robot_xy) < distance_xy(b, robot_xy);
      });

    for (const auto & candidate_xy : candidates) {
      if (is_goal_admissible(grid_map, global_costmap, candidate_xy, config.goal_clearance_radius)) {
        return FrontierTarget{frontier, anchor_xy, candidate_xy};
      }
    }
  }

  return std::nullopt;
}

}  // namespace go2w_real
