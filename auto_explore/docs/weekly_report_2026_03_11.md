# Weekly Report: GO2W Autonomous Frontier Exploration

**Date:** 2026-03-11
**Project:** `go2w_auto_explore` — Real robot autonomous exploration

---

## Objective

Enable the GO2W wheeled quadruped to perform autonomous frontier exploration in real-world environments, replicating the `auto_explore_sim` capability on physical hardware.

---

## Work Completed

### 1. Unitree SLAM Integration Attempt (Abandoned)

**Goal:** Use Unitree's built-in SLAM + navigation (API 1102) for exploration.

**What was built:**
- `slam_client.py` — Pure ROS2 pub/sub client for Unitree SLAM service (`/api/slam_operate/request|response`). Replaced SDK2-based implementation to avoid CycloneDDS in-process conflicts.
- `scan_to_gridmap.py` — Lightweight occupancy grid builder from `/scan` + TF (Bresenham raycast, log-odds). Needed because Unitree SLAM doesn't produce a 2D OccupancyGrid.
- `frontier_explorer.py` — Adapted from `auto_explore_sim`, replaced Nav2 action clients with SlamClient API calls. Added arrival detection via `/slam_key_info` ROS2 topic.
- `official_slam_odom_bridge.py` — Bridges `/unitree/slam_mapping/odom` to `/odom` with TF `map -> lidar`.
- `auto_explore.launch.py` — Full pipeline: static TF, odom bridge, scan pipeline, gridmap builder, frontier explorer.

**Key technical fixes:**
- **CycloneDDS version conflict:** Robot had CycloneDDS 0.7.0 (Foxy apt), 0.10.2 (/usr/local), 0.10.5 (unitree_ros2). Fixed by sourcing `~/unitree_ros2/setup.sh` which overlays rmw_cyclonedds_cpp rebuilt against 0.10.5.
- **SDK2 + ROS2 in-process crash:** `ChannelFactoryInitialize()` + `rclpy.init()` both create CycloneDDS domains, causing `bad_alloc`. Fixed by replacing ALL SDK2 usage with pure ROS2 pub/sub on `/api/slam_operate/*` topics.
- **TF tree conflict:** URDF published `base -> body -> lidar` while odom bridge published `map -> lidar`, creating two parents for `lidar`. Fixed by removing URDF/robot_state_publisher entirely.
- **NavigateTo backoff logic:** Added exponential backoff (3s -> 6s -> 12s -> 30s cap) for transient SLAM errors, distinguishing "SLAM not ready" from "bad goal" failures.

**Why abandoned:**
- Unitree SLAM's NavigateTo (API 1102) returns `errorCode=4 "Failed to obtain current pose"` during active mapping mode.
- **Root cause:** API 1102 only works after relocation (loading a saved PCD map via API 1804), NOT during mapping (API 1801).
- The intended Unitree workflow is: map → save → relocate → navigate. This fundamentally conflicts with simultaneous explore+map.
- Confirmed by testing: after 4+ minutes of mapping with valid odom, NavigateTo still fails consistently.

### 2. SLAM Toolbox + Nav2 Approach

**Goal:** Use SLAM Toolbox for mapping + Nav2 for navigation, reusing proven `auto_explore_sim` frontier explorer.

**Architecture:**
```
xt16_driver (robot)  ->  /unitree/slam_lidar/points
go2w_bridge (robot)  ->  odom->base TF + /odom + cmd_vel conversion
pointcloud_to_laserscan  ->  /scan_raw
laser_filters            ->  /scan
SLAM Toolbox             ->  /map + map->odom TF
Nav2 (DWB controller)    ->  path planning + obstacle avoidance
frontier_explorer        ->  autonomous exploration goals (from auto_explore_sim)
```

**What was built:**
- Rewrote `auto_explore.launch.py` for SLAM Toolbox + Nav2 pipeline
- Created `nav2_params_foxy.yaml` — Foxy-compatible Nav2 config (DWB controller instead of MPPI, `nav2_recoveries` instead of `nav2_behaviors`, no velocity_smoother/collision_monitor)
- Updated `explore_params.yaml` with `robot_base_frame: base`
- Cloned `slam_toolbox` (foxy-devel branch) on robot
- Copied `go2w_real` and `auto_explore_sim` packages to robot

**Status on 2026-03-11:** Initial pipeline assembled, but robot-side validation was still incomplete.

### 3. Robot Debug Session and Final Fixes (Completed 2026-03-12)

**Goal:** Bring up the full all-on-robot pipeline on the actual GO2W and identify why `ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false` did not complete.

**Important environment detail:**
- The launch must be run on the robot, not from the workstation.
- The correct shell environment is:
  - `source /opt/ros/foxy/setup.bash`
  - `source ~/unitree_ros2/setup.sh`
  - `source ~/ros_ws/install/setup.bash`
- `~/unitree_ros2/setup.sh` is required because it provides the CycloneDDS / rmw overlay expected by Unitree SDK2 and the robot's Foxy environment.

**What was failing on the robot:**
- The initial launch never passed the TF gate `odom <- lidar`, so `slam_toolbox` and Nav2 were not started.
- Earlier failed launches had also left stale `slam_toolbox`, scan pipeline, Nav2, and frontier explorer processes running in the background, which made later tests noisy and misleading.

**Root cause 1: robot had an older `go2w_bridge.py` installed**
- The robot's installed `go2w_bridge.py` was not the latest workspace version.
- That older script subscribed only to `rt/sportmodestate`.
- GO2W publishes sport state on `rt/lf/sportmodestate`.
- Result: `go2w_bridge` never received `SportModeState`, so it never published `/odom` or TF `odom -> base`.
- Without `odom -> base`, the launch's TF gate for `odom <- lidar` timed out and intentionally blocked SLAM/Nav2 startup.

**Fix for root cause 1:**
- Replaced the robot-side `go2w_bridge.py` with the newer workspace version.
- The updated bridge:
  - subscribes to both `rt/lf/sportmodestate` and `rt/sportmodestate`
  - treats `network_interface:=auto` correctly instead of forcing an invalid interface string into SDK2
  - logs when the first `SportModeState` packet arrives
- Rebuilt `go2w_real` on the robot.
- Verification:
  - standalone `ros2 run go2w_real go2w_bridge.py` immediately reported `Received first SportModeState on rt/lf/sportmodestate`
  - during full launch, `wait_for_transform.py` reported `Transform ready: odom <- lidar`

**Root cause 2: missing `base -> base_footprint` TF**
- After fixing the bridge, the launch progressed into `slam_toolbox`, but SLAM repeatedly warned:
  - `Invalid frame ID "base_footprint" passed to canTransform`
  - `Failed to compute odom pose`
- The rest of the launch used `base` as the robot frame, but `slam_toolbox` on the robot still expected `base_footprint` somewhere in the TF tree.

**Fix for root cause 2:**
- Added a static identity transform `base -> base_footprint` in `auto_explore.launch.py`.
- Rebuilt `go2w_auto_explore` on the robot.
- After this change, `slam_toolbox` stopped looping on the `base_footprint` warning and was able to register the lidar normally.

**Operational cleanup performed during debugging:**
- Killed orphaned processes from earlier failed runs:
  - `static_transform_publisher`
  - `pointcloud_to_laserscan_node`
  - `scan_to_scan_filter_chain`
  - `async_slam_toolbox_node`
  - Nav2 lifecycle nodes
  - `frontier_explorer_foxy.py`
- This was necessary because old scan / Nav2 / frontier nodes were still consuming CPU and interfering with clean validation.

**Final verified result on 2026-03-12:**
- `go2w_bridge` received `SportModeState` on `rt/lf/sportmodestate`
- `odom -> base` was published
- TF gate `odom <- lidar` passed
- `slam_toolbox` started and registered the lidar
- Nav2 lifecycle manager brought up controller, planner, recoveries, and BT navigator successfully
- `frontier_explorer` started, selected frontiers, and sent accepted goals
- `bt_navigator` began navigating to exploration goals

**Working launch command on the robot:**
```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
source ~/ros_ws/install/setup.bash
ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false
```

**Conclusion:**
- The all-on-robot SLAM Toolbox + Nav2 + frontier explorer stack is now running successfully on the GO2W.
- The key fixes were not algorithmic; they were robot-side integration fixes:
  1. update `go2w_bridge.py` so GO2W sport state is actually consumed
  2. add the missing `base -> base_footprint` transform expected by SLAM
  3. always launch under the correct Foxy + Unitree environment on the robot

---

## Key Findings

| Finding | Detail |
|---------|--------|
| Unitree SLAM NavigateTo only works in relocation mode | API 1102 requires a loaded PCD map, cannot navigate during active mapping |
| SDK2 and ROS2 CycloneDDS conflict in same process | Solved with pure ROS2 pub/sub on `/api/slam_operate/*` topics |
| Robot runs Foxy (20.04) with older Nav2 | No MPPI, no velocity_smoother, no collision_monitor — need DWB controller |
| `unitree_ros2/setup.sh` is required for DDS | Provides CycloneDDS 0.10.5 overlay matching SDK2 |
| xt16_driver runs independently of unitree_slam | Can use pointcloud without running SLAM binary |
| GO2W sport state topic differs from older examples | GO2W publishes `rt/lf/sportmodestate`; subscribing only to `rt/sportmodestate` breaks odom/TF |
| `slam_toolbox` required `base_footprint` in this robot setup | Adding static TF `base -> base_footprint` removed the pose-computation failure |
| Robot-side debugging must be done in the robot shell environment | Running from the workstation hid the real Foxy + Unitree DDS behavior |

---

## Open Questions

1. **Long-run stability:** The stack now launches and explores successfully, but it still needs extended-duration testing to confirm there are no memory, DDS, or lifecycle issues over long missions.
2. **Planner tuning:** Nav2 occasionally reports `failed to create plan with tolerance 0.50` for some frontier goals. This likely needs costmap / frontier filtering / robot footprint tuning rather than launch-level fixes.
3. **Jetson performance margin:** The Jetson handled the verified bringup, but CPU and memory headroom should still be measured during longer exploration runs.

---

## Next Steps

1. Run longer real-world exploration sessions and log map growth, navigation success rate, and recovery behavior
2. Tune Nav2 DWB and costmap parameters to reduce unreachable or low-quality frontier goals
3. Add a documented cleanup/relaunch procedure so stale nodes do not survive failed test runs
4. Record the final robot bringup workflow in package docs so future launches use the correct Foxy + Unitree environment
