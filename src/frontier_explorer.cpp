#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "go2w_real/exploration_types.hpp"
#include "go2w_real/frontier_search.hpp"

namespace
{

struct BlacklistedPoint
{
  double x{0.0};
  double y{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct CompletedPoint
{
  double x{0.0};
  double y{0.0};
};

struct PendingNavigationGoal
{
  double x{0.0};
  double y{0.0};
  std::optional<double> yaw;
  bool returning_to_initial_pose{false};
};

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quat;
  quat.z = std::sin(0.5 * yaw);
  quat.w = std::cos(0.5 * yaw);
  return quat;
}

std::string normalize_frame_id(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
{
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double distance_xy(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace

class FrontierExplorerNode : public rclcpp::Node
{
public:
  using Frontier = go2w_real::Frontier;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavTo = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FrontierExplorerNode()
  : rclcpp::Node("frontier_explorer")
  {
    declare_parameter("planner_frequency", 0.2);
    declare_parameter("min_frontier_size", 20);
    declare_parameter("robot_base_frame", "base");
    declare_parameter("odom_frame", "odom");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("global_frame", "map");
    declare_parameter("map_topic", "/map");
    declare_parameter("transform_tolerance", 2.0);
    declare_parameter("blacklist_radius", 0.5);
    declare_parameter("blacklist_timeout", 40.0);
    declare_parameter("progress_timeout", 30.0);
    declare_parameter("visualize", true);
    declare_parameter("frontier_update_radius", 0.0);
    declare_parameter("return_to_init", false);
    declare_parameter("direct_frontier_goal_search_radius", 2.0);
    declare_parameter("goal_update_min_distance", 0.4);
    declare_parameter("frontier_prev_target_weight", 4.0);
    declare_parameter("frontier_robot_distance_weight", 2.0);
    declare_parameter("frontier_forward_alignment_weight", 1.0);
    declare_parameter("frontier_size_weight", 0.25);

    planner_freq_ = std::max(0.05, get_parameter("planner_frequency").as_double());
    frontier_search_config_.min_frontier_size = std::max(
      1, static_cast<int>(get_parameter("min_frontier_size").as_int()));
    frontier_search_config_.frontier_update_radius = std::max(
      0.0, get_parameter("frontier_update_radius").as_double());
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    blacklist_radius_ = std::max(0.05, get_parameter("blacklist_radius").as_double());
    blacklist_timeout_ = std::max(1.0, get_parameter("blacklist_timeout").as_double());
    progress_timeout_ = std::max(1.0, get_parameter("progress_timeout").as_double());
    visualize_ = get_parameter("visualize").as_bool();
    return_to_init_ = get_parameter("return_to_init").as_bool();
    direct_goal_search_radius_ = std::max(
      0.1, get_parameter("direct_frontier_goal_search_radius").as_double());
    goal_update_min_distance_ = std::max(
      0.05, get_parameter("goal_update_min_distance").as_double());
    frontier_prev_target_weight_ = std::max(
      0.0, get_parameter("frontier_prev_target_weight").as_double());
    frontier_robot_distance_weight_ = std::max(
      0.0, get_parameter("frontier_robot_distance_weight").as_double());
    frontier_forward_alignment_weight_ = std::max(
      0.0, get_parameter("frontier_forward_alignment_weight").as_double());
    frontier_size_weight_ = std::max(
      0.0, get_parameter("frontier_size_weight").as_double());

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    rclcpp::QoS tf_qos(rclcpp::KeepLast(100));
    tf_qos.best_effort();
    tf_qos.durability_volatile();

    rclcpp::QoS tf_static_qos(rclcpp::KeepLast(100));
    tf_static_qos.reliable();
    tf_static_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf_static_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    resume_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&FrontierExplorerNode::resume_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    if (visualize_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "explore/frontiers", 10);
    }

    const auto timer_period =
      std::chrono::duration<double>(1.0 / planner_freq_);
    plan_timer_ = create_wall_timer(
      timer_period, std::bind(&FrontierExplorerNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Frontier explorer ready: map_topic=%s odom_topic=%s goal_search_radius=%.2fm",
      map_topic_.c_str(), odom_topic_.c_str(), direct_goal_search_radius_);
  }

private:
  void timer_callback()
  {
    auto robot_xy = get_robot_position();
    if (robot_xy && !initial_pose_) {
      initial_pose_ = robot_xy;
    }

    if (navigating_ && robot_xy) {
      check_progress(*robot_xy);
    }

    if (!exploring_) {
      return;
    }

    if (!map_msg_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for map data on %s", map_topic_.c_str());
      return;
    }

    if (!robot_xy) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for robot pose (%s -> %s -> %s)",
        global_frame_.c_str(), odom_frame_.c_str(), robot_base_frame_.c_str());
      return;
    }

    prune_blacklist();

    const auto grid = make_grid_view();
    if (!grid.valid()) {
      return;
    }

    std::vector<double> dummy_dist_map(map_msg_->data.size(), 0.0);
    const auto observed_frontiers =
      go2w_real::FrontierSearch::search(grid, *robot_xy, dummy_dist_map, frontier_search_config_);

    std::vector<Frontier> snapped_frontiers;
    snapped_frontiers.reserve(observed_frontiers.size());
    for (const auto & frontier : observed_frontiers) {
      const auto goal_xy = find_navigation_goal(grid, frontier);
      if (!goal_xy) {
        blacklist_point(
          frontier.centroid_x,
          frontier.centroid_y,
          "No free navigation cell near the frontier centroid");
        continue;
      }

      Frontier snapped_frontier = frontier;
      snapped_frontier.centroid_x = goal_xy->first;
      snapped_frontier.centroid_y = goal_xy->second;
      snapped_frontier.min_distance = distance_xy(*robot_xy, *goal_xy);
      snapped_frontier.heuristic_distance = snapped_frontier.min_distance;
      snapped_frontier.cost = snapped_frontier.min_distance;
      snapped_frontiers.push_back(snapped_frontier);
    }

    auto active_frontiers = filter_frontiers(snapped_frontiers);
    if (active_frontiers.empty() && !blacklisted_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "No usable frontier remains after blacklist filtering - clearing blacklist immediately and rescoring");
      blacklisted_.clear();
      active_frontiers = filter_frontiers(snapped_frontiers);
    }

    if (!active_frontiers.empty()) {
      sort_frontiers_for_selection(active_frontiers, *robot_xy);
    }

    if (visualize_) {
      if (active_frontiers.empty()) {
        clear_frontier_markers();
      } else {
        publish_frontier_markers(active_frontiers, active_frontiers.front());
      }
    }

    for (const auto & frontier : active_frontiers) {
      if (frontier.min_distance <= frontier_reached_radius()) {
        complete_frontier(frontier.centroid_x, frontier.centroid_y);
        continue;
      }

      const auto frontier_xy = std::make_pair(frontier.centroid_x, frontier.centroid_y);
      if (navigating_ && !should_preempt_navigation(frontier_xy, frontier_xy)) {
        return;
      }

      if (navigating_) {
        RCLCPP_INFO(
          get_logger(),
          "Preempting frontier goal: (%.2f, %.2f) -> (%.2f, %.2f)",
          current_frontier_ ? current_frontier_->first : std::numeric_limits<double>::quiet_NaN(),
          current_frontier_ ? current_frontier_->second : std::numeric_limits<double>::quiet_NaN(),
          frontier_xy.first,
          frontier_xy.second);
        request_navigation_preempt(
          frontier_xy.first,
          frontier_xy.second,
          compute_goal_yaw(*robot_xy, frontier_xy),
          false);
        return;
      }
      current_frontier_ = frontier_xy;
      navigate_to(frontier_xy.first, frontier_xy.second, compute_goal_yaw(*robot_xy, frontier_xy));
      return;
    }

    if (return_to_init_ && initial_pose_ && !returned_to_initial_pose_) {
      const double dist_home = distance_xy(*robot_xy, *initial_pose_);
      if (dist_home > 0.3) {
        if (navigating_) {
          request_navigation_preempt(initial_pose_->first, initial_pose_->second, std::nullopt, true);
        } else {
          current_frontier_.reset();
          returning_to_initial_pose_ = true;
          navigate_to(initial_pose_->first, initial_pose_->second);
        }
        return;
      }
      returned_to_initial_pose_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No reachable frontier is currently available");
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto frame_id = normalize_frame_id(msg->header.frame_id);
    if (!frame_id.empty() && frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odom pose in unexpected frame '%s'; expected '%s'",
        frame_id.c_str(), odom_frame_.c_str());
      return;
    }

    odom_pose_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    odom_stamp_ = msg->header.stamp;
    update_robot_pose_cache();
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & transform : msg->transforms) {
      const auto parent = normalize_frame_id(transform.header.frame_id);
      const auto child = normalize_frame_id(transform.child_frame_id);
      if (parent == global_frame_ && child == odom_frame_) {
        store_map_to_odom(
          transform.transform.translation.x,
          transform.transform.translation.y,
          yaw_from_quaternion(transform.transform.rotation),
          transform.header.stamp);
      } else if (parent == odom_frame_ && child == global_frame_) {
        const double tx = transform.transform.translation.x;
        const double ty = transform.transform.translation.y;
        const double yaw = yaw_from_quaternion(transform.transform.rotation);
        const double cos_y = std::cos(yaw);
        const double sin_y = std::sin(yaw);
        store_map_to_odom(
          -(cos_y * tx + sin_y * ty),
          sin_y * tx - cos_y * ty,
          -yaw,
          transform.header.stamp);
      }
    }
  }

  void resume_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    exploring_ = msg->data;
    if (!exploring_) {
      RCLCPP_INFO(get_logger(), "Exploration paused");
      return;
    }
    RCLCPP_INFO(get_logger(), "Exploration resumed");
  }

  void nav_feedback_callback(const geometry_msgs::msg::PoseStamped & current_pose)
  {
    const auto frame_id = normalize_frame_id(current_pose.header.frame_id);
    if (!frame_id.empty() && frame_id != global_frame_) {
      return;
    }
    robot_pose_ = std::make_pair(
      current_pose.pose.position.x,
      current_pose.pose.position.y);
    robot_yaw_ = yaw_from_quaternion(current_pose.pose.orientation);
    robot_pose_stamp_ = current_pose.header.stamp;
  }

  void navigate_to(double x, double y, std::optional<double> yaw = std::nullopt)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    if (yaw) {
      goal.pose.pose.orientation = yaw_to_quaternion(*yaw);
    } else {
      goal.pose.pose.orientation.w = 1.0;
    }

    ++goal_seq_;
    const int seq = goal_seq_;

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, seq](GoalHandleNavTo::SharedPtr handle) {
        goal_response_callback(handle, seq);
      };
    options.feedback_callback =
      [this](
      GoalHandleNavTo::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        nav_feedback_callback(feedback->current_pose);
      };
    options.result_callback =
      [this, seq](const GoalHandleNavTo::WrappedResult & result) {
        navigation_result_callback(result, seq);
      };

    nav_client_->async_send_goal(goal, options);

    current_goal_ = std::make_pair(x, y);
    if (!returning_to_initial_pose_ && current_frontier_) {
      last_frontier_target_ = *current_frontier_;
    }
    navigating_ = true;
    cancel_requested_ = false;
    progress_timeout_cancel_requested_ = false;
    last_progress_time_ = now();
    last_robot_pos_ = get_robot_position();

    RCLCPP_INFO(get_logger(), "Sending frontier goal: (%.2f, %.2f)", x, y);
  }

  void request_navigation_preempt(
    double x,
    double y,
    std::optional<double> yaw,
    bool returning_to_initial_pose)
  {
    pending_goal_ = PendingNavigationGoal{x, y, yaw, returning_to_initial_pose};

    if (preempt_requested_ && cancel_requested_) {
      return;
    }

    if (!navigating_) {
      dispatch_pending_goal();
      return;
    }

    preempt_requested_ = true;
    cancel_current_goal();
  }

  void dispatch_pending_goal()
  {
    if (!pending_goal_) {
      return;
    }

    const auto pending_goal = *pending_goal_;
    pending_goal_.reset();
    current_frontier_ = pending_goal.returning_to_initial_pose ?
      std::optional<std::pair<double, double>>{} :
      std::make_optional(std::make_pair(pending_goal.x, pending_goal.y));
    returning_to_initial_pose_ = pending_goal.returning_to_initial_pose;
    navigate_to(pending_goal.x, pending_goal.y, pending_goal.yaw);
  }

  void goal_response_callback(GoalHandleNavTo::SharedPtr handle, int seq)
  {
    if (seq != goal_seq_) {
      return;
    }

    if (!handle) {
      RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
      blacklist_current_frontier("Goal rejected by Nav2");
      clear_navigation_state();
      return;
    }

    nav_goal_handle_ = handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
    if (preempt_requested_ && pending_goal_) {
      cancel_current_goal();
    }
  }

  void navigation_result_callback(const GoalHandleNavTo::WrappedResult & result, int seq)
  {
    if (seq != goal_seq_) {
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (returning_to_initial_pose_) {
        returned_to_initial_pose_ = true;
        RCLCPP_INFO(get_logger(), "Returned to the initial pose");
      } else if (current_frontier_) {
        complete_frontier(current_frontier_->first, current_frontier_->second);
        RCLCPP_INFO(get_logger(), "Frontier goal reached");
      } else {
        RCLCPP_INFO(get_logger(), "Navigation succeeded");
      }
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(get_logger(), "Navigation aborted by Nav2");
      blacklist_current_frontier("Navigation aborted by Nav2");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      if (preempt_requested_) {
        RCLCPP_INFO(get_logger(), "Navigation canceled for goal handoff");
      } else if (progress_timeout_cancel_requested_) {
        RCLCPP_WARN(get_logger(), "Navigation canceled after progress timeout");
      } else {
        RCLCPP_INFO(get_logger(), "Navigation canceled");
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "Navigation ended with result code %d",
        static_cast<int>(result.code));
    }

    const bool should_dispatch_pending = preempt_requested_ && pending_goal_.has_value();
    clear_navigation_state();
    if (should_dispatch_pending) {
      dispatch_pending_goal();
    }
  }

  void check_progress(const std::pair<double, double> & robot_xy)
  {
    if (!last_progress_time_) {
      last_progress_time_ = now();
    }

    if (!last_robot_pos_) {
      last_robot_pos_ = robot_xy;
      return;
    }

    if (distance_xy(robot_xy, *last_robot_pos_) > 0.15) {
      last_robot_pos_ = robot_xy;
      last_progress_time_ = now();
      return;
    }

    if (cancel_requested_ || !last_progress_time_) {
      return;
    }

    const double elapsed = (now() - *last_progress_time_).seconds();
    if (elapsed <= progress_timeout_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "No progress for %.0fs - canceling the current frontier goal",
      elapsed);
    progress_timeout_cancel_requested_ = true;
    blacklist_current_frontier("No progress timeout");
    cancel_current_goal();
  }

  bool should_preempt_navigation(
    const std::pair<double, double> & frontier_xy,
    const std::pair<double, double> & goal_xy) const
  {
    if (!navigating_) {
      return true;
    }

    if (cancel_requested_ && !preempt_requested_) {
      return false;
    }

    if (returning_to_initial_pose_ || !current_frontier_ || !current_goal_) {
      return true;
    }

    return distance_xy(frontier_xy, *current_frontier_) > goal_update_min_distance_ ||
           distance_xy(goal_xy, *current_goal_) > goal_update_min_distance_;
  }

  void cancel_current_goal()
  {
    if (!nav_goal_handle_) {
      if (!preempt_requested_) {
        clear_navigation_state();
      }
      return;
    }
    if (cancel_requested_) {
      return;
    }
    cancel_requested_ = true;
    nav_client_->async_cancel_goal(nav_goal_handle_);
  }

  void clear_navigation_state()
  {
    navigating_ = false;
    cancel_requested_ = false;
    progress_timeout_cancel_requested_ = false;
    preempt_requested_ = false;
    nav_goal_handle_.reset();
    current_goal_.reset();
    current_frontier_.reset();
    returning_to_initial_pose_ = false;
    last_progress_time_.reset();
    last_robot_pos_.reset();
  }

  go2w_real::GridMapView make_grid_view() const
  {
    go2w_real::GridMapView grid;
    if (!map_msg_) {
      return grid;
    }
    grid.cells = &map_msg_->data;
    grid.width = static_cast<int>(map_msg_->info.width);
    grid.height = static_cast<int>(map_msg_->info.height);
    grid.resolution = map_msg_->info.resolution;
    grid.origin_x = map_msg_->info.origin.position.x;
    grid.origin_y = map_msg_->info.origin.position.y;
    return grid;
  }

  void store_map_to_odom(
    double tx,
    double ty,
    double yaw,
    const builtin_interfaces::msg::Time & stamp)
  {
    map_to_odom_xy_ = std::make_pair(tx, ty);
    map_to_odom_yaw_ = yaw;
    map_to_odom_stamp_ = stamp;
    update_robot_pose_cache();
  }

  void update_robot_pose_cache()
  {
    if (!map_to_odom_xy_ || !map_to_odom_yaw_ || !odom_pose_) {
      return;
    }

    const double cos_y = std::cos(*map_to_odom_yaw_);
    const double sin_y = std::sin(*map_to_odom_yaw_);
    robot_pose_ = std::make_pair(
      map_to_odom_xy_->first + cos_y * odom_pose_->first - sin_y * odom_pose_->second,
      map_to_odom_xy_->second + sin_y * odom_pose_->first + cos_y * odom_pose_->second);
    if (odom_yaw_) {
      robot_yaw_ = *map_to_odom_yaw_ + *odom_yaw_;
    }
    robot_pose_stamp_ = odom_stamp_;
  }

  std::optional<std::pair<double, double>> get_robot_position() const
  {
    return robot_pose_;
  }

  std::optional<double> compute_goal_yaw(
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy) const
  {
    const double dx = goal_xy.first - robot_xy.first;
    const double dy = goal_xy.second - robot_xy.second;
    if (std::hypot(dx, dy) < 1e-6) {
      return std::nullopt;
    }
    return std::atan2(dy, dx);
  }

  std::optional<std::pair<double, double>> find_navigation_goal(
    const go2w_real::GridMapView & grid,
    const Frontier & frontier) const
  {
    if (!grid.valid()) {
      return std::nullopt;
    }

    const int center_x = static_cast<int>(
      std::floor((frontier.centroid_x - grid.origin_x) / grid.resolution));
    const int center_y = static_cast<int>(
      std::floor((frontier.centroid_y - grid.origin_y) / grid.resolution));
    if (!grid.in_bounds(center_x, center_y)) {
      return std::nullopt;
    }

    std::optional<std::pair<double, double>> best_goal;
    double best_dist = std::numeric_limits<double>::infinity();
    const int search_radius_cells = std::max(
      1, static_cast<int>(std::ceil(direct_goal_search_radius_ / grid.resolution)));

    for (int dy = -search_radius_cells; dy <= search_radius_cells; ++dy) {
      for (int dx = -search_radius_cells; dx <= search_radius_cells; ++dx) {
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (!grid.in_bounds(x, y) || grid.value(x, y) != 0) {
          continue;
        }
        const auto world = grid.cell_center_world(x, y);
        const double dist = std::hypot(
          world.first - frontier.centroid_x,
          world.second - frontier.centroid_y);
        if (dist < best_dist) {
          best_dist = dist;
          best_goal = world;
        }
      }
    }

    return best_goal;
  }

  double frontier_reached_radius() const
  {
    return std::max(0.35, 2.0 * blacklist_radius_);
  }

  std::vector<Frontier> filter_frontiers(const std::vector<Frontier> & snapped_frontiers) const
  {
    std::vector<Frontier> active_frontiers;
    active_frontiers.reserve(snapped_frontiers.size());
    for (const auto & frontier : snapped_frontiers) {
      if (is_blacklisted(frontier.centroid_x, frontier.centroid_y) ||
        is_completed(frontier.centroid_x, frontier.centroid_y))
      {
        continue;
      }
      active_frontiers.push_back(frontier);
    }
    return active_frontiers;
  }

  std::optional<std::pair<double, double>> selection_reference_target() const
  {
    if (current_frontier_) {
      return current_frontier_;
    }
    return last_frontier_target_;
  }

  void sort_frontiers_for_selection(
    std::vector<Frontier> & frontiers,
    const std::pair<double, double> & robot_xy) const
  {
    if (frontiers.empty()) {
      return;
    }

    const auto reference_target = selection_reference_target();
    const double prev_weight = reference_target ? frontier_prev_target_weight_ : 0.0;
    const double alignment_weight = robot_yaw_ ? frontier_forward_alignment_weight_ : 0.0;

    double max_reference_distance = 0.0;
    double max_robot_distance = 0.0;
    int max_size = 0;
    for (const auto & frontier : frontiers) {
      if (reference_target) {
        max_reference_distance = std::max(
          max_reference_distance,
          distance_xy(
            std::make_pair(frontier.centroid_x, frontier.centroid_y),
            *reference_target));
      }
      max_robot_distance = std::max(max_robot_distance, frontier.heuristic_distance);
      max_size = std::max(max_size, frontier.size);
    }

    for (auto & frontier : frontiers) {
      double prev_target_score = 0.0;
      if (prev_weight > 0.0) {
        if (max_reference_distance <= 1e-6) {
          prev_target_score = 1.0;
        } else {
          prev_target_score = 1.0 - clamp01(
            distance_xy(
              std::make_pair(frontier.centroid_x, frontier.centroid_y),
              *reference_target) / max_reference_distance);
        }
      }

      const double robot_distance_score =
        max_robot_distance <= 1e-6 ? 1.0 :
        1.0 - clamp01(frontier.heuristic_distance / max_robot_distance);

      double alignment_score = 0.5;
      if (alignment_weight > 0.0) {
        const double heading_to_frontier = std::atan2(
          frontier.centroid_y - robot_xy.second,
          frontier.centroid_x - robot_xy.first);
        alignment_score = 0.5 * (std::cos(heading_to_frontier - *robot_yaw_) + 1.0);
      }

      const double size_score =
        max_size <= 0 ? 1.0 :
        static_cast<double>(frontier.size) / static_cast<double>(max_size);

      frontier.cost =
        prev_weight * prev_target_score +
        frontier_robot_distance_weight_ * robot_distance_score +
        alignment_weight * alignment_score +
        frontier_size_weight_ * size_score;
    }

    std::sort(
      frontiers.begin(), frontiers.end(),
      [](const Frontier & a, const Frontier & b) {
        if (a.cost != b.cost) {
          return a.cost > b.cost;
        }
        if (a.heuristic_distance != b.heuristic_distance) {
          return a.heuristic_distance < b.heuristic_distance;
        }
        return a.size > b.size;
      });
  }

  void blacklist_current_frontier(const std::string & reason)
  {
    if (!current_frontier_) {
      return;
    }
    blacklist_point(current_frontier_->first, current_frontier_->second, reason);
  }

  void blacklist_point(double x, double y, const std::string & reason)
  {
    if (is_blacklisted(x, y)) {
      return;
    }
    blacklisted_.push_back({x, y, now()});
    RCLCPP_WARN(
      get_logger(),
      "Blacklisting frontier (%.2f, %.2f): %s",
      x, y, reason.c_str());
  }

  void complete_frontier(double x, double y)
  {
    if (is_completed(x, y)) {
      return;
    }
    completed_.push_back({x, y});
  }

  bool is_blacklisted(double x, double y) const
  {
    for (const auto & item : blacklisted_) {
      if (std::hypot(item.x - x, item.y - y) <= blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  bool is_completed(double x, double y) const
  {
    for (const auto & item : completed_) {
      if (std::hypot(item.x - x, item.y - y) <= frontier_reached_radius()) {
        return true;
      }
    }
    return false;
  }

  void prune_blacklist()
  {
    const auto now_time = now();
    std::vector<BlacklistedPoint> retained;
    retained.reserve(blacklisted_.size());
    for (const auto & item : blacklisted_) {
      if ((now_time - item.stamp).seconds() < blacklist_timeout_) {
        retained.push_back(item);
      }
    }
    blacklisted_ = std::move(retained);
  }

  void clear_frontier_markers()
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    marker_pub_->publish(markers);
  }

  void publish_frontier_markers(
    const std::vector<Frontier> & frontiers,
    const Frontier & chosen)
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    const auto stamp = now();
    int marker_id = 1;
    for (const auto & frontier : frontiers) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = "frontiers";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = frontier.centroid_x;
      marker.pose.position.y = frontier.centroid_y;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.18;
      marker.scale.y = 0.18;
      marker.scale.z = 0.18;

      const bool selected =
        std::hypot(
        frontier.centroid_x - chosen.centroid_x,
        frontier.centroid_y - chosen.centroid_y) <= 1e-4;
      if (selected) {
        marker.color.g = 1.0;
      } else {
        marker.color.r = 0.2;
        marker.color.g = 0.4;
        marker.color.b = 1.0;
      }
      marker.color.a = 0.85;
      marker.lifetime = rclcpp::Duration::from_seconds(5.0);
      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  go2w_real::FrontierSearchConfig frontier_search_config_;

  double planner_freq_{0.2};
  std::string robot_base_frame_{"base"};
  std::string odom_frame_{"odom"};
  std::string odom_topic_{"/odom"};
  std::string global_frame_{"map"};
  std::string map_topic_{"/map"};
  double transform_tolerance_{2.0};
  double blacklist_radius_{0.5};
  double blacklist_timeout_{40.0};
  double progress_timeout_{30.0};
  bool visualize_{true};
  bool return_to_init_{false};
  double direct_goal_search_radius_{2.0};
  double goal_update_min_distance_{0.4};
  double frontier_prev_target_weight_{4.0};
  double frontier_robot_distance_weight_{2.0};
  double frontier_forward_alignment_weight_{1.0};
  double frontier_size_weight_{0.25};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavTo::SharedPtr nav_goal_handle_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  bool exploring_{true};
  bool navigating_{false};
  bool cancel_requested_{false};
  bool progress_timeout_cancel_requested_{false};
  bool returning_to_initial_pose_{false};
  bool returned_to_initial_pose_{false};
  bool preempt_requested_{false};
  int goal_seq_{0};

  std::optional<std::pair<double, double>> odom_pose_;
  std::optional<double> odom_yaw_;
  builtin_interfaces::msg::Time odom_stamp_;
  std::optional<std::pair<double, double>> map_to_odom_xy_;
  std::optional<double> map_to_odom_yaw_;
  builtin_interfaces::msg::Time map_to_odom_stamp_;
  std::optional<std::pair<double, double>> robot_pose_;
  std::optional<double> robot_yaw_;
  builtin_interfaces::msg::Time robot_pose_stamp_;
  std::optional<std::pair<double, double>> current_frontier_;
  std::optional<std::pair<double, double>> current_goal_;
  std::optional<std::pair<double, double>> last_frontier_target_;
  std::optional<PendingNavigationGoal> pending_goal_;
  std::optional<std::pair<double, double>> last_robot_pos_;
  std::optional<rclcpp::Time> last_progress_time_;
  std::optional<std::pair<double, double>> initial_pose_;

  std::vector<BlacklistedPoint> blacklisted_;
  std::vector<CompletedPoint> completed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
