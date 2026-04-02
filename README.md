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
ros2 launch go2w_real slam_rf2o.launch.py
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
- [`scripts/save_map_and_waypoints.py`](scripts/save_map_and_waypoints.py): fixed waypoint recorder
- [`scripts/navigate_to_waypoint.py`](scripts/navigate_to_waypoint.py): terminal waypoint selector that sends goals to Nav2
- [`scripts/profile_nav_stack.py`](scripts/profile_nav_stack.py): CPU profiler for the SLAM + Nav2 runtime stack

## Network Setup

Connect the PC to the robot through Ethernet and put the PC on the robot subnet.

Example:

```bash
ip link
sudo ip addr add 192.168.123.100/24 dev eth0
```

Robot default IP is commonly `192.168.123.161`.

## Quick Start: Stable Mapping Workflow

### 1. Start the XT16 lidar driver

Terminal 1, start the XT16 lidar driver first:

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

### 2. Source the environment

Terminal 2:

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
```

### 3. Make sure the lidar is publishing

```bash
ros2 topic list | grep unitree/slam_lidar/points
ros2 topic hz /unitree/slam_lidar/points
```

Expected input topic:

- `/unitree/slam_lidar/points`

### 4. Launch SLAM + Nav2

```bash
ros2 launch go2w_real slam_rf2o.launch.py
```

Optional overrides:

```bash
ros2 launch go2w_real slam_rf2o.launch.py \
  cloud_topic:=/unitree/slam_lidar/points \
  use_rviz:=true
```

Notes:

- `network_interface` defaults to `eth0`, so you do not need to pass it in the normal case
- only add `network_interface:=...` when your robot is connected through a different NIC
- `slam_params.yaml` currently loads the serialized SLAM map prefix `/home/unitree/ros_ws/src/map/test_1`

### 5. Optional: inspect the saved waypoint table

Terminal 3:

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py --list-pois
ros2 run go2w_real navigate_to_waypoint.py --list-only
```

These two commands only print the current POI / waypoint table and then exit.
They do not start navigation by themselves.

### 6. Build the initial map with the Unitree remote

Do not use ROS teleop in the mature mapping workflow unless you explicitly want it.

Recommended practice:

- keep speed low
- avoid aggressive spinning in place
- prefer smooth loops and revisits
- let SLAM close loops naturally

### 7. Test a navigation goal in RViz

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

### 8. Save fixed navigation points

Run the recorder in another terminal:

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real save_map_and_waypoints.py
```

Then:

- use RViz `Publish Point` to click a point on the map
- the script records the clicked position first
- the script records the robot's current heading next
- enter the point name in the terminal last
- repeat for all fixed points
- press `Ctrl+C` when finished

The script will:

- save waypoint metadata only
- write the YAML to `/home/unitree/ros_ws/src/go2w_waypoints.yaml` by default

Generated files:

- `/home/unitree/ros_ws/src/go2w_waypoints.yaml`

### 9. Navigate to a saved waypoint from the terminal

The waypoint navigator reads a waypoint YAML file, shows all available targets,
and sends the selected target to Nav2's `/navigate_to_pose` action.

Default input file:

- `/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml`

List available waypoints only:

```bash
ros2 run go2w_real navigate_to_waypoint.py --list-only
```

Start the interactive selector:

```bash
ros2 run go2w_real navigate_to_waypoint.py
```

Then:

- review the printed waypoint list
- enter either the waypoint index, POI id, or waypoint name
- the script sends that pose to Nav2
- the terminal prints navigation feedback until the goal finishes
- enter another waypoint, or `q` to quit

Send one waypoint directly without the prompt:

```bash
ros2 run go2w_real navigate_to_waypoint.py --waypoint POI_003
```

If your waypoint YAML is stored elsewhere:

```bash
ros2 run go2w_real navigate_to_waypoint.py \
  --waypoint-file /home/unitree/ros_ws/src/go2w_waypoints.yaml
```

### 10. Run as the WebSocket navigation executor

Use the executor when the navigation manager sends `navigate_to` /
`abort_navigation` commands over WebSocket and expects Nav2 execution feedback.

Chinese usage guide:

- [`docs/navigation_executor_zh.md`](docs/navigation_executor_zh.md)
- [`docs/navigation_sender_zh.md`](docs/navigation_sender_zh.md)

Default endpoint:

- `ws://192.168.0.131:8100/ws/navigation/executor`

Start it with the default waypoint table:

```bash
ros2 run go2w_real navigation_executor.py
```

Point it to a remote manager:

```bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://192.168.0.131:8100/ws/navigation/executor
```

List the POI table the executor will resolve:

```bash
ros2 run go2w_real navigation_executor.py --list-pois
```

Behavior summary:

- `target_id` matches waypoint `id` first
- if `id` is absent, it falls back to waypoint `name`
- numeric `target_id` values also match the waypoint index
- a new `navigate_to` command preempts the current goal
- `abort_navigation` pauses the current goal and leaves the executor ready for the next command
- heartbeat `ping` / `pong` and automatic reconnect are built in

### 11. CPU profiling for SLAM + Nav2

Use the built-in profiler when you want a quick CPU breakdown of the running
stack without manually hunting through `htop`.

```bash
ros2 run go2w_real profile_nav_stack.py
```

The profiler groups CPU usage by runtime component, including:

- `go2w_bridge`
- `pointcloud_to_laserscan`
- `laser_filters`
- `rf2o_laser_odometry`
- `slam_toolbox`
- `controller_server`
- `planner_server`
- `recoveries_server`
- `bt_navigator`
- `rviz2`
- `navigate_to_waypoint`
- `navigation_executor`

Useful options:

```bash
ros2 run go2w_real profile_nav_stack.py --interval 0.5 --show-missing
ros2 run go2w_real profile_nav_stack.py --samples 20
```

## Waypoint File Format

Waypoints are stored as YAML with:

- optional serialized map metadata
- optional POI id
- point name
- frame id
- position
- yaw
- quaternion orientation

Example:

```yaml
map:
  serialized_prefix: '/home/unitree/ros_ws/src/map/test_1'
  data: '/home/unitree/ros_ws/src/map/test_1.data'
  posegraph: '/home/unitree/ros_ws/src/map/test_1.posegraph'
waypoints:
  - id: 'POI_001'
    name: 'dock'
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
ros2 launch go2w_real slam_rf2o.launch.py
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

### Need a custom waypoint YAML path

```bash
ros2 run go2w_real save_map_and_waypoints.py \
  --waypoint-file /home/unitree/ros_ws/src/my_waypoints.yaml
```

## Repository Notes

- `go2w_auto_explore` has been split into its own top-level package
- `rf2o_laser_odometry` is included in the workspace and patched for Foxy compatibility
- the active script set is now focused on the RF2O + slam_toolbox + Nav2 workflow

## License

MIT
