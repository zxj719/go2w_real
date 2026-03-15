# Weekly Report: GO2W Autonomous Frontier Exploration — Sim2Real

**Date:** 2026-03-15
**Project:** `go2w_auto_explore` — Real robot autonomous SLAM + Nav2 exploration

---

## Objective

Port the C++ frontier explorer from `go2w_office_sim` to the real GO2W robot running ROS2 Foxy, and resolve navigation safety issues for autonomous exploration in real-world environments.

---

## Work Completed

### 1. C++ Frontier Explorer Port (from go2w_office_sim)

**Goal:** Replace the Python `frontier_explorer_foxy.py` with the C++ `frontier_explorer.cpp` from `go2w_office_sim/src/` for better performance on the Jetson.

**What was done:**
- Ported ~1500-line C++ frontier explorer with BFS frontier detection, GVD (Generalized Voronoi Diagram) skeleton extraction, and A* path planning on GVD
- Created Foxy compatibility layer with `#ifdef ROS_FOXY_COMPAT`:
  - `goal_response_callback` uses `std::shared_future<GoalHandle::SharedPtr>` (Foxy) vs `GoalHandle::SharedPtr` (Humble+)
  - `rclcpp::Duration(30, 0)` instead of `rclcpp::Duration::from_seconds(30.0)`
  - Removed `const` from methods calling `Clock::now()` (non-const in Foxy)
- Removed all `NavigateThroughPoses` code (not available in Foxy, added in Galactic) — always uses `NavigateToPose` with single goal
- GVD path still computed for visualization but only the final waypoint is sent as a nav goal
- Updated `CMakeLists.txt` with ROS distro detection for automatic Foxy compat flags
- Updated `package.xml` with C++ dependencies (rclcpp, rclcpp_action, nav2_msgs, etc.)
- Updated launch file to use C++ executable instead of Python script

**Build verification:** Compiles on both Jazzy (workstation, without `ROS_FOXY_COMPAT`) and Foxy (robot, with `ROS_FOXY_COMPAT`).

### 2. DWB Obstacle Avoidance Fix — Robot Hitting Walls

**Problem:** Robot moved aggressively and collided with walls during exploration. Had to physically pull the robot to prevent damage.

**Root cause:** The DWB local planner's obstacle critic `BaseObstacle` was configured with `scale: 0.02`, which is ~1000x weaker than the path-following critics (`PathAlign.scale: 32.0`, `GoalDist.scale: 24.0`). The robot essentially ignored obstacles in trajectory selection.

**Fix applied in `nav2_params_foxy.yaml`:**
- Replaced `BaseObstacle` (scale 0.02) with `ObstacleFootprint` (scale 10.0) — uses actual robot footprint for collision checking
- Reduced max velocities: `max_vel_x` 0.35 → 0.25 m/s, `max_vel_theta` 1.5 → 1.0 rad/s
- Reduced accelerations: `acc_lim_x` 1.5 → 1.0 m/s²
- Increased trajectory simulation time: `sim_time` 1.7 → 2.0s (longer lookahead)
- Tuned inflation layer: `inflation_radius` 0.55m, `cost_scaling_factor` 3.0

**Result:** Robot now respects obstacle clearance during trajectory selection.

### 3. RViz Costmap & Path Visualization

**Goal:** Add costmap visualization to debug navigation behavior.

**Added to `auto_explore.rviz`:**
- GlobalCostmap (`/global_costmap/costmap`) with costmap color scheme, alpha 0.4
- LocalCostmap (`/local_costmap/costmap`) with costmap color scheme, alpha 0.5
- NavPlan (`/plan`) — global path in green
- LocalPlan (`/local_plan`) — local trajectory in yellow
- SlamOdom (`/odom`) — odometry arrows for heading verification

### 4. 90-Degree Heading Mismatch Diagnosis & Fix

**Problem:** In RViz, the odom heading arrow pointed 90 degrees LEFT of the robot's actual physical heading. This caused the robot to drive INTO obstacles when trying to avoid them — the harder it tried to escape, the more it collided.

**Root cause analysis:**
- The `go2w_auto_explore` launch uses `/unitree/slam_lidar/points` from **xt16_driver** (frame: `rslidar`)
- The `go2w_real` nav2 stack uses `/unilidar/cloud` from the **unilidar SDK** (different driver)
- These two LiDAR drivers have different coordinate axis conventions
- The `lidar -> rslidar` static TF was identity (no rotation), so the xt16_driver's X-axis was misaligned with the robot's forward direction by 90 degrees
- SLAM Toolbox built a map with the scan rotated 90 degrees, causing the entire coordinate system to be misaligned

**Fix applied:**
- Added configurable `lidar_yaw_offset` launch argument (default: `1.5708` = π/2)
- Applied to `lidar -> rslidar` static TF: `arguments=["0", "0", "0", lidar_yaw_offset, "0", "0", "lidar", "rslidar"]`
- Can be overridden at launch: `lidar_yaw_offset:=1.5708` or `-1.5708` for tuning

**Launch command:**
```bash
ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false lidar_yaw_offset:=1.5708
```

---

## Current Configuration Summary

| Parameter | Value |
|-----------|-------|
| Controller | DWB (`dwb_core::DWBLocalPlanner`) |
| Planner | NavfnPlanner (A*: off) |
| Max linear speed | 0.25 m/s |
| Max angular speed | 1.0 rad/s |
| Obstacle critic | `ObstacleFootprint` (scale 10.0) |
| Inflation radius | 0.55 m |
| Goal tolerance (xy) | 0.7 m |
| Goal tolerance (yaw) | 3.14 rad (any heading) |
| LiDAR yaw offset | 1.5708 rad (π/2) |
| Frontier min size | 20 cells |
| Explore frequency | 0.2 Hz |

---

## Key Findings

| Finding | Detail |
|---------|--------|
| xt16_driver vs unilidar SDK axis convention | Different LiDAR drivers have different X-axis orientations; must compensate with static TF rotation |
| DWB `BaseObstacle` vs `ObstacleFootprint` | `BaseObstacle` at low scale is essentially disabled; `ObstacleFootprint` with proper scale is required for real-world safety |
| C++ Foxy vs Humble API differences | `goal_response_callback` signature, `Duration::from_seconds`, const `Clock::now()` all differ |
| `NavigateThroughPoses` not in Foxy | Must use single-goal `NavigateToPose` only |
| xt16_driver must be started before launch | If not running, entire scan pipeline produces no data; no error — silent failure |

---

## Files Modified This Week

| File | Change |
|------|--------|
| `src/frontier_explorer.cpp` | **NEW** — C++ frontier explorer ported from go2w_office_sim, Foxy-compatible |
| `CMakeLists.txt` | Added C++ targets, ROS distro detection for Foxy compat |
| `package.xml` | Added C++ dependencies |
| `config/nav2_params_foxy.yaml` | ObstacleFootprint critic, reduced speeds, tuned inflation |
| `config/explore_params.yaml` | Updated to match sim config with profiling |
| `launch/auto_explore.launch.py` | C++ explorer, `lidar_yaw_offset` parameter |
| `rviz/auto_explore.rviz` | Added costmap, path, odom visualization |

---

## Open Questions

1. **LiDAR yaw offset sign:** Default is π/2 (1.5708 rad). May need testing with -π/2 if heading is still misaligned in some configurations.
2. **Long-run stability:** Extended exploration sessions needed to validate map quality, recovery behavior, and Jetson resource usage.
3. **Frontier quality near obstacles:** Some frontiers may be too close to walls; `min_frontier_size` and costmap padding may need further tuning.

---

## Next Steps

1. Validate heading alignment with extended real-world exploration runs
2. Tune frontier selection to prefer safer, more open frontiers
3. Test recovery behaviors (spin, backup, wait) in tight spaces
4. Measure Jetson CPU/memory during 30+ minute exploration sessions
5. Consider switching to `/unilidar/cloud` (direct LiDAR SDK) to avoid xt16_driver dependency and axis convention issues
