# go2w_real

ROS 2 package for deploying SLAM + Nav2 navigation on the **real Unitree GO2W** (wheeled quadruped) robot. Bridges ROS 2 `cmd_vel` to the Unitree SDK2 `SportClient` and publishes odometry, IMU, and TF from the robot's `SportModeState`.

This is the **sim2real** counterpart of [`go2w_office_sim`](https://github.com/zxj719/go2w_office_sim), sharing the same Nav2/SLAM stack with parameters tuned for real hardware.

## Architecture

```
                          ┌─────────────────────────────────────────┐
                          │            Real GO2W Robot              │
                          │  ┌──────────┐  ┌──────────────────┐    │
                          │  │ L2 LiDAR │  │ SportModeState   │    │
                          │  │ (4D,360x │  │ (odom,imu,vel)   │    │
                          │  │  96 FOV) │  │                  │    │
                          │  └────┬─────┘  └────────┬─────────┘    │
                          │       │PointCloud2      │ DDS          │
                          └───────┼─────────────────┼──────────────┘
                                  │                 │
                    ┌─────────────┼─────────────────┼──────────────────┐
                    │ ROS 2       │                 │                  │
                    │             ▼                 ▼                  │
                    │  pointcloud_to_laserscan   go2w_bridge           │
                    │    /unilidar/cloud           /odom, /tf          │
                    │         │                   /imu/data            │
                    │         ▼                      │                 │
                    │    /scan_raw                   │                 │
                    │         │                      │                 │
                    │    laser_filters               │                 │
                    │    (self-occlusion)             │                 │
                    │         │                      │                 │
                    │         ▼                      │                 │
                    │      /scan ──────► SLAM Toolbox ◄── /odom       │
                    │         │              │                         │
                    │         │         map->odom TF                   │
                    │         │              │                         │
                    │         ▼              ▼                         │
                    │       Nav2 Stack (MPPI + SmacPlanner2D)          │
                    │              │                                   │
                    │         /cmd_vel                                 │
                    │              │                                   │
                    │              ▼                                   │
                    │        go2w_bridge                               │
                    │     SportClient.Move(vx,vy,vyaw)                │
                    └─────────────────────────────────────────────────┘
```

## Scan Pipeline

```
L2 4D LiDAR (PointCloud2)     Gazebo 3D gpu_lidar (PointCloud2)
  /unilidar/cloud         or     /pointcloud_raw
         │                              │
         ▼                              ▼
   pointcloud_to_laserscan (height filter: -0.1m ~ 0.3m)
         │
    /scan_raw (LaserScan)
         │
    laser_filters (LaserScanBoxFilter, removes robot body self-hits)
         │
      /scan (LaserScan) → SLAM Toolbox / Nav2 costmaps / collision monitor
```

## Prerequisites

- ROS 2 Foxy
- `unitree_sdk2_python` (in workspace)
- `pointcloud_to_laserscan`: `sudo apt install ros-foxy-pointcloud-to-laserscan`
- `laser_filters`: `sudo apt install ros-foxy-laser-filters`
- Nav2, SLAM Toolbox (standard ROS 2 nav stack)

## Network Setup

Connect your PC to GO2W via Ethernet. The robot's default IP is `192.168.123.161`. Configure your PC to be on the same subnet:

```bash
# Check your interface name
ip link
# Set IP (example)
sudo ip addr add 192.168.123.100/24 dev eth0
```

## Usage

### 1. Basic Bringup (teleop)

```bash
ros2 launch go2w_real bringup.launch.py network_interface:=eth0

# Auto stand_up is enabled by default.
# Disable it if needed:
ros2 launch go2w_real bringup.launch.py network_interface:=eth0 auto_stand_up:=false

# Or trigger stand_up manually:
ros2 run go2w_real go2_motion_command.py --action stand_up --network-interface eth0
ros2 topic pub /cmd_control std_msgs/String '{data: stand_up}' --once

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. SLAM + Nav2 (full navigation)

```bash
ros2 launch go2w_real nav2.launch.py network_interface:=eth0

# Auto stand_up is enabled by default.
# Disable it if needed:
ros2 launch go2w_real nav2.launch.py network_interface:=eth0 auto_stand_up:=false

# The default LiDAR source now matches the working auto_explore pipeline:
#   /unitree/slam_lidar/points
# Override it if your robot publishes a different topic:
ros2 launch go2w_real nav2.launch.py network_interface:=eth0 \
  cloud_topic:=/unitree/slam_lidar/points

# Disable RViz on a headless robot:
ros2 launch go2w_real nav2.launch.py network_interface:=eth0 use_rviz:=false

# Set 2D Goal in RViz to navigate
```

### 3. Official Unitree SLAM + Nav2 (no slam_toolbox)

If official Unitree SLAM is already running on the robot and publishing:
`/global_map`, `/unitree/slam_mapping/odom`, `/unitree/slam_lidar/points`

```bash
ros2 launch go2w_real nav2_official_slam.launch.py network_interface:=eth0
```

### 4. SLAM Mapping with RF2O lidar odometry

If Unitree's odometer service is unavailable, use lidar-only odometry:

```bash
ros2 launch go2w_real slam_rf2o.launch.py

# Override the pointcloud topic if needed:
ros2 launch go2w_real slam_rf2o.launch.py \
  cloud_topic:=/unitree/slam_lidar/points
```

This launch assumes the robot is already standing and you are driving with the
Unitree remote.

### 5. Control Commands

```bash
# Python SDK one-shot helper
ros2 run go2w_real go2_motion_command.py --action stand_up --network-interface eth0
ros2 run go2w_real go2_motion_command.py --action recovery --network-interface eth0

# Stand up / down
ros2 topic pub /cmd_control std_msgs/String '{data: stand_up}' --once
ros2 topic pub /cmd_control std_msgs/String '{data: stand_down}' --once

# Emergency stop
ros2 topic pub /cmd_control std_msgs/String '{data: damp}' --once

# Recovery (if robot falls)
ros2 topic pub /cmd_control std_msgs/String '{data: recovery}' --once
```

## Workspace Layout

```text
ros_ws/src/
├── go2w_real/
│   ├── scripts/
│   │   ├── go2w_bridge.py          # Core: cmd_vel <-> SportClient, state -> odom/tf/imu
│   │   ├── go2_motion_command.py   # One-shot SDK helper (stand_up, recovery, etc.)
│   │   └── wait_for_transform.py   # Gates SLAM/Nav2 startup on a valid TF chain
│   ├── config/
│   │   ├── nav2_params.yaml        # Nav2 (conservative speeds for real hardware)
│   │   ├── slam_params.yaml        # SLAM Toolbox
│   │   └── laser_filter.yaml       # Self-occlusion box filter
│   ├── launch/
│   │   ├── bringup.launch.py       # Bridge + RSP + lidar pipeline (teleop-ready)
│   │   ├── nav2.launch.py          # Full: bridge + SLAM + Nav2 + RViz
│   │   ├── nav2_official_slam.launch.py
│   │   ├── slam_mapping.launch.py  # Mapping with SDK/sport-state odom
│   │   └── slam_rf2o.launch.py     # Mapping with lidar-derived RF2O odom
│   ├── rviz/
│   │   └── nav2_real.rviz          # RViz layout: robot, map, scan, TF, odom
│   ├── urdf/
│   │   ├── go2w_real.urdf.xacro    # TF tree (no Gazebo plugins)
│   │   └── const.xacro
│   └── meshes/                     # Local GO2W meshes; package is self-contained
└── go2w_auto_explore/
    ├── launch/auto_explore.launch.py
    ├── config/
    ├── docs/
    ├── rviz/
    ├── scripts/
    ├── src/
    └── urdf/
```

`go2w_auto_explore` is now a separate top-level package in the workspace, not a subdirectory under `go2w_real`.

## Key Differences from Simulation

| Aspect | Sim (`go2w_office_sim`) | Real (`go2w_real`) |
|--------|------------------------|-------------------|
| Actuation | Gazebo DiffDrive plugin | `SportClient.Move()` via SDK DDS |
| Odometry | Gazebo ground-truth | `SportModeState` (SDK) |
| LiDAR | Gazebo 3D gpu_lidar | Unitree L2 4D LiDAR |
| LiDAR topic | `/pointcloud_raw` | `/unilidar/cloud` |
| local_costmap frame | `map` (TF timestamp bug) | `odom` (no bug on real hw) |
| `vx_max` | 0.5 m/s | 0.35 m/s (safety) |
| Max accelerations | 2.5 / 3.2 | 1.5 / 2.5 (gentler) |

## Unitree L2 4D LiDAR Specs

| Spec | Value |
|------|-------|
| FOV | 360 x 96 (horizontal x vertical) |
| Range | 0.05 - 30 m |
| Point rate | 64,000 pts/s |
| Rotation speed | 5.55 Hz |
| Accuracy | <= 2 cm |
| Communication | Ethernet UDP |
| Weight | 230 g |
| Built-in IMU | 3-axis accel + 3-axis gyro |

## License

MIT
