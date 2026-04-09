#pragma once

#include <cstddef>

namespace go2w_real
{

inline bool should_recover_blacklist(
  std::size_t active_frontier_count,
  std::size_t blacklisted_count)
{
  return blacklisted_count > 0 && active_frontier_count == 0;
}

}  // namespace go2w_real
