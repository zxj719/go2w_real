#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "go2w_real/exploration_types.hpp"

namespace go2w_real
{

struct FrontierGoalSelectorConfig
{
  double snap_radius{1.0};
  double fallback_snap_radius{1.0};
  double goal_clearance_radius{0.35};
};

struct FrontierTarget
{
  Frontier frontier;
  std::pair<double, double> anchor_xy;
  std::pair<double, double> goal_xy;
};

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double suppression_radius);

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const FrontierGoalSelectorConfig & config);

}  // namespace go2w_real
