#pragma once

namespace go2w_real
{

inline bool should_blacklist_on_navigation_abort(
  bool preempt_requested,
  bool has_pending_goal)
{
  return !(preempt_requested && has_pending_goal);
}

inline bool should_accept_new_frontier_goal_while_navigating(bool navigating)
{
  (void)navigating;
  return true;
}

inline bool should_retry_same_frontier_after_failure(
  bool has_current_frontier,
  bool retry_used,
  bool has_retry_goal)
{
  (void)has_current_frontier;
  (void)retry_used;
  (void)has_retry_goal;
  return false;
}

}  // namespace go2w_real
