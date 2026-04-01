# go2w_real

ROS 2 package for mapping and navigation on the real Unitree GO2W.

The current SLAM + navigation test workflow in this repository is considered stable for real-robot use:

- LiDAR source: `/unitree/slam_lidar/points`
- 2D scan generation: `pointcloud_to_laserscan` + `laser_filters`
- odometry for mapping: `rf2o_laser_odometry`
- SLAM backend: `slam_toolbox`
- robot driving during mapping: Unitree remote control

This path avoids depending on Unitree's odometer service, which may exist on some machines but was not consistently publishing usable odometry in our tested setup.

## Status

Recommended unified launch:

```bash
ros2 launch go2w_real slam_rf2o.launch.py network_interface:=eth0
```

This is the recommended path for:

- manual map recording
- RViz visualization
- RViz `2D Goal Pose` navigation testing
- later waypoint collection on the finished map

`slam_rf2o.launch.py` is the default entrypoint for this package.

## Tested Data Flow

The mature mapping stack is:

```text
/unitree/slam_lidar/points
  -> pointcloud_relay.py
  -> /cloud_relayed
  -> pointcloud_to_laserscan
  -> /scan_raw
  -> laser_filters
  -> /scan
  -> rf2o_laser_odometry
  -> /odom and odom->base TF
  -> slam_toolbox
  -> /map and map->odom TF
```

Useful observed topics in the tested setup:

- `/unitree/slam_lidar/points`: stable and suitable as the mapping point cloud input
- `/utlidar/imu`: stable and published at high rate
- `/utlidar/robot_odom`: topic may exist but was not reliably publishing usable samples
- `/utlidar/robot_pose`: topic may exist but was not reliably publishing usable samples
- `rt/lf/sportmodestate`: reachable through SDK, but in our tests often reported zero position and velocity during mapping work

Because of that, lidar-derived odometry is the preferred mapping source here.

## Requirements

- Ubuntu + ROS 2 Foxy
- `pointcloud_to_laserscan`
- `laser_filters`
- `slam_toolbox`
- `nav2_map_server`
- `rf2o_laser_odometry`
- Unitree ROS 2 environment sourced when needed

Typical install commands:

```bash
sudo apt install ros-foxy-pointcloud-to-laserscan
sudo apt install ros-foxy-laser-filters
sudo apt install ros-foxy-slam-toolbox
sudo apt install ros-foxy-nav2-map-server
```

Build:

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select rf2o_laser_odometry go2w_real
source install/setup.bash
```

## Workspace Layout

```text
ros_ws/src/
├── go2w_real/
│   ├── config/
│   ├── launch/
│   ├── meshes/
│   ├── rviz/
│   ├── scripts/
│   └── urdf/
├── go2w_auto_explore/
└── rf2o_laser_odometry/
```

Important files:

- [`launch/slam_rf2o.launch.py`](launch/slam_rf2o.launch.py): unified RF2O + slam_toolbox + Nav2 launch
- [`rviz/nav2_real.rviz`](rviz/nav2_real.rviz): RViz layout with `2D Goal Pose`, map, costmaps, and plans
- [`scripts/save_map_and_waypoints.py`](scripts/save_map_and_waypoints.py): map + waypoint recorder

## Network Setup

Connect the PC to the robot through Ethernet and put the PC on the robot subnet.

Example:

```bash
ip link
sudo ip addr add 192.168.123.100/24 dev eth0
```

Robot default IP is commonly `192.168.123.161`.

## Quick Start: Stable Mapping Workflow

### 1. Source the environment

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
```

### 2. Make sure the lidar is publishing

```bash
ros2 topic list | grep unitree/slam_lidar/points
ros2 topic hz /unitree/slam_lidar/points
```

Expected input topic:

- `/unitree/slam_lidar/points`

### 3. Launch SLAM + Nav2

```bash
ros2 launch go2w_real slam_rf2o.launch.py network_interface:=eth0
```

Optional overrides:

```bash
ros2 launch go2w_real slam_rf2o.launch.py \
  network_interface:=eth0 \
  cloud_topic:=/unitree/slam_lidar/points \
  use_rviz:=true
```

### 4. Build the initial map with the Unitree remote

Do not use ROS teleop in the mature mapping workflow unless you explicitly want it.

Recommended practice:

- keep speed low
- avoid aggressive spinning in place
- prefer smooth loops and revisits
- let SLAM close loops naturally

### 5. Test a navigation goal in RViz

Wait until RViz shows all of these before testing:

- robot model on the map
- `/map`
- global path and costmaps when Nav2 finishes startup

Then:

- click `2D Goal Pose` in the RViz top toolbar
- click once on a free-space point in the map
- drag to set the goal yaw and release
- watch `/plan`, `/local_plan`, `/global_costmap/costmap`, and `/local_costmap/costmap`
- if you want to stop immediately, publish a zero command or cancel the Nav2 goal

### 6. Save the map and fixed navigation points

Run the recorder in another terminal:

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real save_map_and_waypoints.py --map-prefix ~/maps/go2w_map
```

Then:

- use RViz `Publish Point` to click a point on the map
- enter the point name in the terminal
- enter yaw in degrees, or press Enter to use the robot's current heading
- repeat for all fixed points
- press `Ctrl+C` when finished

The script will:

- save the occupancy map
- save waypoint metadata

Generated files:

- `~/maps/go2w_map.yaml`
- `~/maps/go2w_map.pgm`
- `~/maps/go2w_map_waypoints.yaml`

## Waypoint File Format

Waypoints are stored as YAML with:

- point name
- frame id
- position
- yaw
- quaternion orientation

Example:

```yaml
map:
  prefix: '/home/unitree/maps/go2w_map'
  yaml: '/home/unitree/maps/go2w_map.yaml'
  image: '/home/unitree/maps/go2w_map.pgm'
waypoints:
  - name: 'dock'
    frame_id: 'map'
    position:
      x: 1.250000
      y: -0.480000
      z: 0.000000
    yaw: 1.570796
    orientation:
      x: 0.000000
      y: 0.000000
      z: 0.707107
      w: 0.707107
```

## Launch Files

### Unified SLAM + Nav2 entrypoint

```bash
ros2 launch go2w_real slam_rf2o.launch.py network_interface:=eth0
```

Use this when:

- you want reliable real-world map building
- you want RViz `2D Goal Pose` testing on the live map
- official odometer service is unavailable or inconsistent
- you are manually driving with the remote before sending Nav2 goals

## Known Good Assumptions

The current mapping recommendation assumes:

- the robot is already standing
- the remote control is used for driving
- `/unitree/slam_lidar/points` is available
- RViz can run locally, or `use_rviz:=false` is used on headless systems

## Troubleshooting

### No map growth in RViz

Check:

```bash
ros2 topic hz /unitree/slam_lidar/points
ros2 topic hz /scan
ros2 topic hz /odom
```

You need all three to move during mapping:

- point cloud in
- scan out
- odom out

### RViz starts but robot or map is missing

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base
ros2 run tf2_ros tf2_echo map odom
```

### Official odom topics exist but have no samples

This was observed in the tested setup. Topic names may appear, but usable odometry may still not be published. That is why RF2O is the recommended default.

### Need to save only waypoints, not the map

```bash
ros2 run go2w_real save_map_and_waypoints.py \
  --map-prefix ~/maps/go2w_map \
  --no-save-map
```

## Repository Notes

- `go2w_auto_explore` has been split into its own top-level package
- `rf2o_laser_odometry` is included in the workspace and patched for Foxy compatibility
- `go2w_bridge.py` still supports SDK-based motion / state bridging for other bringup flows

## License

MIT
