#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "go2w_real/exploration_types.hpp"

namespace go2w_real
{

struct FrontierSearchConfig
{
  int min_frontier_size{5};
  double clearance_scale{0.3};
  double frontier_update_radius{3.0};
  double frontier_distance_weight{1.0};
  double frontier_size_weight{0.0};
};

class FrontierSearch
{
public:
  static std::vector<Frontier> search(
    const GridMapView & grid,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map,
    const FrontierSearchConfig & config);
};

}  // namespace go2w_real
