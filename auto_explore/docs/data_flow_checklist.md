# GO2W Auto Explore — Data Flow Checklist

SLAM Toolbox + Nav2 + Frontier Explorer on Foxy (all-on-robot)

---

## 1. TF Tree

```
map  ──(slam_toolbox)──▶  odom  ──(go2w_bridge)──▶  base  ──(static)──▶  body  ──(static)──▶  lidar  ──(static)──▶  rslidar
                                                       │
                                                       └──(static)──▶  imu
```

| Parent → Child | Publisher | Type | Offset |
|---------------|-----------|------|--------|
| `map` → `odom` | slam_toolbox | dynamic | SLAM correction |
| `odom` → `base` | go2w_bridge | dynamic | SDK2 SportModeState position + orientation |
| `base` → `body` | static_transform_publisher | static | identity (0,0,0) |
| `body` → `lidar` | static_transform_publisher | static | (0.28945, 0, -0.046825) |
| `lidar` → `rslidar` | static_transform_publisher | static | identity (xt16_driver uses frame `rslidar`) |

### Checklist
- [ ] `odom → base` published at 50 Hz (go2w_bridge odom_timer)
- [ ] `map → odom` published by slam_toolbox (once SLAM starts processing scans)
- [ ] All static TFs visible: `ros2 run tf2_ros tf2_echo base lidar`
- [ ] Full chain works: `ros2 run tf2_ros tf2_echo map lidar`

---

## 2. Scan Pipeline

```
xt16_driver                pointcloud_to_laserscan           laser_filters
 (XT-16 LiDAR)            (3D→2D slice)                     (self-occlusion removal)
      │                          │                                 │
      ▼                          ▼                                 ▼
/unitree/slam_lidar/points ──▶ /scan_raw ────────────────────▶ /scan
   PointCloud2                  LaserScan                      LaserScan
   frame: rslidar               frame: lidar                   frame: lidar
```

| Topic | Msg Type | Frame | Publisher | Rate |
|-------|----------|-------|-----------|------|
| `/unitree/slam_lidar/points` | PointCloud2 | `rslidar` | xt16_driver | ~10 Hz |
| `/scan_raw` | LaserScan | `lidar` | pointcloud_to_laserscan | ~10 Hz |
| `/scan` | LaserScan | `lidar` | laser_filters | ~10 Hz |

### pointcloud_to_laserscan params
- `target_frame`: lidar
- `min_height`: -0.2 m, `max_height`: 0.4 m (floor obstacle slice)
- `range_min`: 0.05 m, `range_max`: 30.0 m
- `angle_min/max`: ±π (full 360°)

### laser_filters (LaserScanBoxFilter)
- Frame: `lidar`
- Box: x=[-0.6, 0.0], y=[-0.22, 0.22], z=[-1.0, 1.0]
- Removes points from robot body/legs behind the LiDAR mount

### Checklist
- [ ] xt16_driver running: `ps aux | grep xt16`
- [ ] PointCloud2 arriving: `ros2 topic hz /unitree/slam_lidar/points`
- [ ] Raw scan output: `ros2 topic hz /scan_raw`
- [ ] Filtered scan output: `ros2 topic hz /scan`
- [ ] No self-occlusion in /scan (compare /scan_raw vs /scan in RViz)

---

## 3. Odometry (go2w_bridge)

```
Unitree SDK2 (DDS)                go2w_bridge                    ROS2
rt/lf/sportmodestate
  (fallback: rt/sportmodestate) ───────▶  child process (SDK2)  ──Queue──▶  parent process (rclpy)
  SportModeState_               ChannelSubscriber                  │
                                                                   ├──▶ /odom (Odometry)
                                                                   ├──▶ /tf (odom→base)
                                                                   └──▶ /imu/data (Imu)

ROS2                                go2w_bridge
/cmd_vel (Twist) ───────────▶  parent process  ──Queue──▶  child process
/cmd_control (String)             rclpy sub                 SportClient.Move()
```

| Topic | Msg Type | Direction | Rate |
|-------|----------|-----------|------|
| `/odom` | nav_msgs/Odometry | publish | 50 Hz |
| `/tf` (odom→base) | tf2_msgs/TFMessage | publish | 50 Hz |
| `/imu/data` | sensor_msgs/Imu | publish | 50 Hz |
| `/cmd_vel` | geometry_msgs/Twist | subscribe | from Nav2 (20 Hz) |
| `/cmd_control` | std_msgs/String | subscribe | manual (stand_up, stand_down, etc.) |

### Architecture (CycloneDDS conflict workaround)
- **Child process**: SDK2 `ChannelFactoryInitialize()` + `SportClient` + `ChannelSubscriber`
- **Parent process**: `rclpy.init()` + ROS2 pubs/subs
- **IPC**: `multiprocessing.Queue` (state_queue: child→parent, cmd_queue: parent→child)
- **Critical**: Child must fork BEFORE `rclpy.init()` or CycloneDDS domain conflict

### Checklist
- [ ] Robot in sport mode (SportModeState publishing on `rt/lf/sportmodestate` or `rt/sportmodestate`)
- [ ] go2w_bridge shows "GO2W Bridge ready (multiprocessing)"
- [ ] No "create domain error" in logs
- [ ] Odom publishing: `ros2 topic hz /odom`
- [ ] TF working: `ros2 run tf2_ros tf2_echo odom base`
- [ ] cmd_vel safety timeout: robot stops if no cmd_vel for 0.5s

---

## 4. SLAM Toolbox

```
/scan (LaserScan) ──────▶  slam_toolbox  ──────▶  /map (OccupancyGrid)
TF: odom→base     ──────▶  (online_async)  ────▶  TF: map→odom
```

| Input | Output |
|-------|--------|
| `/scan` (LaserScan, frame: lidar) | `/map` (OccupancyGrid, frame: map) |
| TF: `odom → base` | TF: `map → odom` |
| | `/map_metadata` (MapMetaData) |

### Key params (slam_params.yaml)
- `base_frame`: base
- `odom_frame`: odom
- `map_frame`: map
- `scan_topic`: /scan
- `resolution`: 0.05 m
- `max_laser_range`: 20.0 m
- `minimum_travel_distance`: 0.2 m
- `minimum_travel_heading`: 0.05 rad
- `map_update_interval`: 2.0 s
- `transform_publish_period`: 0.02 s
- Loop closure: enabled (search_distance=3.0m)

### Checklist
- [ ] No "Message Filter dropping" errors (means TF chain is complete)
- [ ] Map publishing: `ros2 topic hz /map`
- [ ] map→odom TF exists: `ros2 run tf2_ros tf2_echo map odom`
- [ ] Map grows as robot moves (check in RViz)

---

## 5. Nav2 Stack (Foxy)

```
                    ┌─────────────────┐
                    │   bt_navigator   │
                    │  (BehaviorTree)  │
                    └───────┬─────────┘
                            │
              ┌─────────────┼─────────────┐
              ▼             ▼             ▼
      ┌──────────────┐ ┌──────────┐ ┌───────────────┐
      │ planner_server│ │controller│ │recoveries_srv │
      │  (NavfnPlanner)│ │(DWB)    │ │(spin/backup/  │
      │               │ │         │ │ wait)          │
      └──────┬────────┘ └────┬────┘ └───────────────┘
             │               │
             ▼               ▼
      global_costmap    local_costmap
      (map frame)       (odom frame)
             │               │
             ▼               ▼
           /scan           /scan
           /map
```

### Topics consumed
| Topic | Consumer | Purpose |
|-------|----------|---------|
| `/scan` | local_costmap, global_costmap | obstacle detection |
| `/map` | global_costmap (static_layer) | static map from SLAM |
| `/odom` | bt_navigator | robot state |
| TF: full chain | all costmaps | coordinate transforms |

### Topics produced
| Topic | Producer | Purpose |
|-------|----------|---------|
| `/cmd_vel` | controller_server (DWB) | velocity commands → go2w_bridge |
| `/plan` | planner_server | global path visualization |
| `/local_costmap/costmap` | local_costmap | obstacle map |
| `/global_costmap/costmap` | global_costmap | planning map |

### Actions (used by frontier_explorer)
| Action | Server | Client |
|--------|--------|--------|
| `/navigate_to_pose` | bt_navigator | frontier_explorer |
| `/compute_path_to_pose` | planner_server | bt_navigator (internal) |
| `/follow_path` | controller_server | bt_navigator (internal) |

### DWB Controller params
- max_vel_x: 0.35 m/s, max_vel_theta: 1.5 rad/s
- acc_lim_x: 1.5, acc_lim_theta: 2.5
- sim_time: 1.7 s
- xy_goal_tolerance: 0.7 m, yaw_goal_tolerance: 3.14 rad

### Costmap params
| | Local | Global |
|---|---|---|
| global_frame | odom | map |
| size | 3×3 m (rolling) | full map |
| resolution | 0.05 m | 0.05 m |
| obstacle_min_range | 0.3 m | 0.3 m |
| inflation_radius | 0.70 m | 0.70 m |
| robot_radius | 0.35 m | 0.25 m |

### Checklist
- [ ] Lifecycle manager reports all nodes active
- [ ] No bt_navigator errors (BT XML loaded, action servers found)
- [ ] Local costmap updating: `ros2 topic hz /local_costmap/costmap`
- [ ] Global costmap has map data: `ros2 topic echo /global_costmap/costmap --once`
- [ ] DWB producing cmd_vel when goal is active

---

## 6. Frontier Explorer

```
/map (OccupancyGrid) ──────▶  frontier_explorer  ──────▶  /navigate_to_pose (Action Goal)
TF: map→base         ──────▶  (BFS frontier      ──────▶  /explore/frontiers (MarkerArray)
                                search + GVD)      ──────▶  /explore/gvd_path (MarkerArray)
```

| Input | Output |
|-------|--------|
| `/map` (OccupancyGrid) | NavigateToPose action goal |
| TF: `map → base` (robot position) | `/explore/frontiers` (visualization) |
| | `/explore/gvd_path` (visualization) |

### Key params (explore_params.yaml)
- `planner_frequency`: 0.5 Hz (replan every 2s)
- `min_frontier_size`: 20 cells
- `robot_base_frame`: base
- `global_frame`: map
- `progress_timeout`: 30 s (cancel stuck navigation)
- `blacklist_timeout`: 120 s (forget failed frontiers)
- `blacklist_radius`: 0.5 m
- `clearance_scale`: 0.2 (GVD bonus weight)
- `gvd_snap_radius`: 2.0 m
- `return_to_init`: false

### Control topics
| Topic | Msg | Purpose |
|-------|-----|---------|
| `/explore/resume` | std_msgs/Bool | pause (false) / resume (true) exploration |

### Checklist
- [ ] "Frontier Explorer ready" in logs
- [ ] Not stuck on "Waiting for map..." (needs /map from slam_toolbox)
- [ ] Frontiers detected and goals sent to Nav2
- [ ] Visualization markers in RViz (/explore/frontiers)
- [ ] Robot moves to frontier goals autonomously

---

## 7. Overall Startup Sequence

```
t=0s   static TFs (base→body→lidar→rslidar)
t=0s   go2w_bridge (SDK2 child fork → rclpy.init → odom/tf/imu)
t=0s   pointcloud_to_laserscan + laser_filters (/scan pipeline)
t=0s   slam_toolbox (waits for /scan + odom→base TF)
t=5s   Nav2 nodes (controller, planner, recoveries, bt_navigator)
t=5s   lifecycle_manager (configures + activates Nav2 in order)
t=15s  frontier_explorer (waits for /map + navigate_to_pose action)
```

### Full System Checklist

**Prerequisites:**
- [ ] Robot powered on and in sport mode
- [ ] xt16_driver running (`cd /unitree/module/unitree_slam/bin && ./xt16_driver`)
- [ ] Network: 192.168.123.x (robot .18, LiDAR .20)

**Launch:**
```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
cd ~/ros_ws && source install/setup.bash
ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false
```

**Post-launch checks:**
- [ ] go2w_bridge: "GO2W Bridge ready" (no CycloneDDS error)
- [ ] `/odom` publishing at 50 Hz
- [ ] `/scan` publishing at ~10 Hz
- [ ] slam_toolbox: no "Message Filter dropping" (TF chain complete)
- [ ] `/map` publishing
- [ ] lifecycle_manager: all nodes active (no "Failed to bring up")
- [ ] frontier_explorer: "Frontier Explorer ready"

**Stand up robot:**
```bash
ros2 topic pub /cmd_control std_msgs/String '{data: stand_up}' --once
```

**Verify exploration:**
- [ ] Frontiers detected in logs
- [ ] NavigateToPose goals sent to Nav2
- [ ] `/cmd_vel` being published (DWB controller active)
- [ ] Robot moving autonomously
- [ ] Map growing in RViz

---

## 8. Troubleshooting Quick Reference

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| "create domain error" | SDK2 forked after rclpy.init | Ensure fork happens BEFORE rclpy.init() |
| "Message Filter dropping frame 'lidar'" | odom→base TF missing | Check go2w_bridge DDS interface + SportModeState topic, robot must be in sport mode |
| "Waiting for map..." | slam_toolbox not producing /map | Check scan pipeline + TF chain |
| "Failed to change state: bt_navigator" | BT XML not found or plugin missing | Use absolute path to BT XML |
| No /cmd_vel output | Nav2 not active or no goal | Check lifecycle_manager, frontier_explorer |
| Robot doesn't move | cmd_vel not reaching SDK2 | Check go2w_bridge cmd_queue, sport mode |
