# GO2W Auto SLAM Guideline

How to start the real robot SLAM stack and verify that the robot is actually mapping.

This guide is for the all-on-robot setup:
- robot: GO2W
- ROS distro: Foxy
- SLAM: `slam_toolbox`
- navigation: Nav2
- exploration: `frontier_explorer`
- LiDAR driver: Unitree `xt16_driver`

---

## 1. Connect to the Robot

From the workstation:

```bash
ssh unitree@192.168.123.18
```

If password is needed:

```bash
123
```

---

## 2. Source the Correct Environment

This must be done on the robot before starting the stack.

```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
source ~/ros_ws/install/setup.bash
```

Why this matters:
- `~/unitree_ros2/setup.sh` provides the Unitree CycloneDDS / RMW overlay
- without it, `go2w_bridge.py` and DDS communication may fail

---

## 3. Put the Robot in a Valid State

Before starting SLAM:
- robot should be powered on
- robot should be reachable on `eth0`
- robot should be in sport mode
- XT16 LiDAR should be connected

Optional check:

```bash
ip -br a
```

Expected:
- `eth0` should be up
- robot IP is usually on `192.168.123.x`

---

## 4. Stop Old Processes First

Do this before every fresh run.

```bash
pkill -f 'go2w_bridge.py|async_slam_toolbox_node|pointcloud_relay.py|pointcloud_to_laserscan_node|scan_to_scan_filter_chain|static_transform_publisher|wait_for_transform.py|controller_server|planner_server|recoveries_server|bt_navigator|lifecycle_manager|frontier_explorer_foxy.py|xt16_driver'
```

If needed, run it twice.

---

## 5. Start the XT16 LiDAR Driver

The driver is the scan source. Without it, SLAM will start but no map will be built.

Run on the robot:

```bash
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./xt16_driver
```

Expected early log lines:

```text
network: eth0
ip_address: 192.168.123.20
lidar_frame: rslidar
lidar_topic: rt/unitree/slam_lidar/points
Load correction file from lidar succeed
```

If you want it in the background:

```bash
nohup bash -lc 'cd /unitree/module/unitree_slam/bin && export LD_LIBRARY_PATH=/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH && ./xt16_driver' > ~/xt16_driver_latest.log 2>&1 < /dev/null &
echo $! > ~/xt16_driver_latest.pid
```

---

## 6. Start the Auto SLAM Stack

In another robot shell:

```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
source ~/ros_ws/install/setup.bash
ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false
```

If you want it in the background:

```bash
nohup bash -lc 'source /opt/ros/foxy/setup.bash && source ~/unitree_ros2/setup.sh && source ~/ros_ws/install/setup.bash && ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false' > ~/auto_slam_latest.log 2>&1 < /dev/null &
echo $! > ~/auto_slam_latest.pid
```

---

## 7. What Success Looks Like

These are the important log messages.

From `go2w_bridge.py`:

```text
[go2w_bridge] Subscribing SportModeState topics: rt/lf/sportmodestate, rt/sportmodestate
[go2w_bridge] Received first SportModeState on rt/lf/sportmodestate
```

From the TF gate:

```text
Transform ready: odom <- lidar
```

From `slam_toolbox`:

```text
Using solver plugin solver_plugins::CeresSolver
Registering sensor: [Custom Described Lidar]
```

From Nav2:

```text
Managed nodes are active
```

From the explorer:

```text
Frontier Explorer ready
Starting frontier exploration...
Sending goal: ...
Goal accepted
```

---

## 8. Verify That the Robot Is Really SLAMing

Do not trust only that the launch started. Check the data flow.

### 8.1 LiDAR data exists

```bash
ros2 topic info /unitree/slam_lidar/points
ros2 topic info /scan
```

Expected:
- `/unitree/slam_lidar/points` has `Publisher count: 1`
- `/scan` has `Publisher count: 1`

### 8.2 Odom is available

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base
```

Expected:
- `/odom` publishes continuously
- `odom -> base` transform exists

### 8.3 Map is being produced

```bash
ros2 topic info /map
ros2 topic echo --once /map_metadata
ros2 run tf2_ros tf2_echo map odom
```

Expected:
- `/map` has `Publisher count: 1`
- `/map_metadata` prints valid width / height / resolution
- `map -> odom` transform exists

### 8.4 Explorer is no longer waiting for map

If the log says only:

```text
Waiting for map...
```

then SLAM is not producing a usable map yet.

### 8.5 Navigation is active

```bash
ros2 node list | grep -E 'slam_toolbox|planner_server|controller_server|bt_navigator|frontier_explorer'
```

Expected nodes:
- `/slam_toolbox`
- `/planner_server`
- `/controller_server`
- `/bt_navigator`
- `/frontier_explorer`

---

## 9. Fast Troubleshooting

### Problem: launch starts but no map appears

Check:

```bash
ros2 topic info /unitree/slam_lidar/points
ros2 topic info /scan
ros2 topic info /map
```

Typical cause:
- `xt16_driver` is not running

Fix:
- start `/unitree/module/unitree_slam/bin/xt16_driver`

### Problem: `odom <- lidar` TF gate never passes

Likely causes:
- robot is not in sport mode
- `go2w_bridge.py` is not receiving `SportModeState`

Check launch log for:

```text
Received first SportModeState on rt/lf/sportmodestate
```

### Problem: DDS / Unitree SDK communication fails

Usually caused by wrong shell environment.

Fix:

```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
source ~/ros_ws/install/setup.bash
```

### Problem: planner reports many `failed to create plan with tolerance 0.50`

This does not always mean the stack is broken.
It usually means:
- current frontier is unreachable
- map is still sparse
- explorer will blacklist that goal and try another one

If mapping is active and the robot keeps receiving new goals, the system is still functioning.

---

## 10. Useful Background-Run Logs

If started with `nohup`, use:

```bash
tail -f ~/xt16_driver_latest.log
tail -f ~/auto_slam_latest.log
```

PID files:

```bash
cat ~/xt16_driver_latest.pid
cat ~/auto_slam_latest.pid
```

---

## 11. Stop Everything Cleanly

If you used the PID files:

```bash
kill $(cat ~/auto_slam_latest.pid)
kill $(cat ~/xt16_driver_latest.pid)
```

Then clean leftovers:

```bash
pkill -f 'go2w_bridge.py|async_slam_toolbox_node|pointcloud_relay.py|pointcloud_to_laserscan_node|scan_to_scan_filter_chain|static_transform_publisher|wait_for_transform.py|controller_server|planner_server|recoveries_server|bt_navigator|lifecycle_manager|frontier_explorer_foxy.py|xt16_driver'
```

---

## 12. Minimal Start Recipe

If you only need the shortest working sequence:

```bash
# terminal 1 on robot
cd /unitree/module/unitree_slam/bin
export LD_LIBRARY_PATH=/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./xt16_driver
```

```bash
# terminal 2 on robot
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/setup.sh
source ~/ros_ws/install/setup.bash
ros2 launch go2w_auto_explore auto_explore.launch.py use_rviz:=false
```

Then confirm:
- `Received first SportModeState on rt/lf/sportmodestate`
- `Transform ready: odom <- lidar`
- `/map` exists
- `Frontier Explorer ready`

