# GO2W XT16 与新 SLAM 算法对接说明

## 1. 背景

当前机器人导航链路使用：

```text
/unitree/slam_lidar/points
  -> /cloud_relayed
  -> /scan_raw
  -> /scan
  -> rf2o_laser_odometry
  -> /odom
  -> slam_toolbox
```

这条链路在静态或低动态环境中可以工作，但当周围动态障碍物明显增多时，RF2O 基于连续 2D 激光帧匹配得到的 `/odom` 容易受到动态点影响，进而导致 `slam_toolbox` 定位漂移，严重时会丢失定位。

新 SLAM 算法团队对接时，建议优先使用 XT16 原始点云以及官方/机身状态相关的 IMU、里程计数据，不要把当前 RF2O 生成的 `/odom` 视为可靠真值。

注意：XT16 雷达本身没有 IMU。需要 IMU 数据时，应订阅机器人 `/lowstate` 话题并读取其中的 `imu_state` 字段。

## 2. 环境准备

机器人与工控机通过网线连接，工控机网卡需要在机器人网段。例如：

```bash
ip link
sudo ip addr add 192.168.123.100/24 dev eth0
```

常见机器人 IP：

```text
192.168.123.161
```

ROS 2 环境：

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
```

如果只验证 XT16 原始话题，通常先启动 `xt16_driver` 即可。

## 3. 启动 XT16

在机器人本机或工控机终端启动：

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

后台启动并保存日志的方式：

```bash
mkdir -p ~/ros_ws/log/xt16
setsid /unitree/module/unitree_slam/bin/xt16_driver \
  > ~/ros_ws/log/xt16/xt16_driver.log 2>&1 &
```

确认进程仍在运行：

```bash
pgrep -af xt16_driver
```

停止：

```bash
pkill -f xt16_driver
```

## 4. XT16 原始话题

启动 `xt16_driver` 后，重点检查下面的话题。

Unitree 官方 LiDAR service 文档中会提到 `/utlidar/cloud_deskewed`、`/utlidar/cloud` 等话题。当前现场调试结果是：ROS 2 图里可以看到 `/utlidar/cloud_deskewed` 端点，RViz2 也已经订阅它，但 `ros2 topic echo`/`ros2 topic hz` 没有收到样本；`/utlidar/cloud` 则可以收到有效 `PointCloud2` 数据，消息 frame 是 `utlidar_lidar`。当前工程自己的 XT16 链路仍以 `/unitree/slam_lidar/points` 为默认输入，但这个话题需要单独启动 `xt16_driver` 后才会出现。

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/unitree/slam_lidar/points` | `sensor_msgs/msg/PointCloud2` | 当前工程默认 XT16 点云输入；需要启动 `/unitree/module/unitree_slam/bin/xt16_driver` |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 官方 LiDAR service 文档中的去畸变点云；当前现场有端点但无样本输出，不建议依赖 |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 当前现场可收到有效点云，frame_id 为 `utlidar_lidar` |
| `/utlidar/robot_odom` | 以现场 `ros2 topic info` 为准 | 可能存在，但当前现场 `ros2 topic echo /utlidar/robot_odom` 没有样本输出，不建议作为新 SLAM 输入 |
| `/utlidar/robot_pose` | 以现场 `ros2 topic info` 为准 | 可能存在，但可用性需要现场确认，不建议默认依赖 |

检查命令：

```bash
ros2 topic list | grep -E 'slam_lidar|utlidar'
ros2 topic list | grep -E 'cloud|points'
ros2 topic info /unitree/slam_lidar/points -v
ros2 topic hz /unitree/slam_lidar/points
ros2 topic echo /unitree/slam_lidar/points --once
```

如果下面命令没有输出，说明当前现场没有这个官方文档中提到的话题：

```bash
ros2 topic info /utlidar/cloud_deskewed -v
ros2 topic echo /utlidar/cloud_deskewed --once
```

当前现场调试命令显示 `/utlidar/cloud` 有数据：

```bash
ros2 topic echo /utlidar/cloud sensor_msgs/msg/PointCloud2
```

典型字段：

```text
frame_id: utlidar_lidar
height: 1
width: 1900-2100
fields: x, y, z, intensity, ring, time
```

如果 RViz2 看不到 `/utlidar/cloud`，先把 Fixed Frame 临时设为 `utlidar_lidar`。当前现场没有 `map`、`odom`、`base` 到 `utlidar_lidar` 的 TF 时，Fixed Frame 设为 `map`/`odom`/`base` 都会导致点云无法显示。

IMU 不来自 XT16。请检查机器人 `/lowstate`，并从消息里的 `imu_state` 字段读取姿态、角速度和加速度：

```bash
ros2 topic info /lowstate -v
ros2 topic hz /lowstate
ros2 topic echo /lowstate --once
```

对接代码中需要读取的字段是：

```text
/lowstate.imu_state
```

官方里程计/位姿检查。当前现场已观察到 `/utlidar/robot_odom` 执行 `ros2 topic echo` 没有输出，因此这里只作为排查项，不作为推荐输入：

```bash
ros2 topic info /utlidar/robot_odom -v
ros2 topic hz /utlidar/robot_odom
ros2 topic echo /utlidar/robot_odom --once

ros2 topic info /utlidar/robot_pose -v
ros2 topic hz /utlidar/robot_pose
ros2 topic echo /utlidar/robot_pose --once
```

注意：如果 `/utlidar/robot_odom` 或 `/utlidar/robot_pose` 没有发布样本，不能作为新 SLAM 的输入。当前现场应按“无可用 `/utlidar/robot_odom`”处理。

## 5. 当前工程生成的话题

如果启动当前仓库的完整 SLAM/Nav2 链路：

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash

ros2 launch go2w_real slam_rf2o.launch.py \
  slam_mode:=mapping \
  use_rviz:=true
```

会产生以下中间话题：

| 话题 | 消息类型 | 来源 | 说明 |
| --- | --- | --- | --- |
| `/cloud_relayed` | `sensor_msgs/msg/PointCloud2` | `pointcloud_relay.py` | 将 `/unitree/slam_lidar/points` 从 RELIABLE QoS 转为下游更容易消费的 BEST_EFFORT |
| `/scan_raw` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | 点云转 2D scan |
| `/scan` | `sensor_msgs/msg/LaserScan` | `laser_filters` | 过滤机器人身体/腿部自遮挡后的 scan |
| `/odom` | `nav_msgs/msg/Odometry` | 默认 RF2O | 当前导航用局部里程计，不建议新 SLAM 团队当作官方里程计 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` | 当前 SLAM 输出地图 |

当前 RF2O 默认订阅 `/scan`，发布 `/odom` 和 `odom -> base` TF。动态障碍物增多时，漂移主要来自这个基于激光匹配的 `/odom`。

## 6. 机身 IMU/里程计

官方原始 IMU 数据应从 `/lowstate` 话题的 `imu_state` 字段获取。当前工程的 `go2w_bridge.py` 还会从 Unitree SDK2 的 `SportModeState` 读取机身状态，并发布 ROS 2 标准消息。完整 `slam_rf2o.launch.py` 中将其重映射为：

| 话题 | 消息类型 | 来源 |
| --- | --- | --- |
| `/lowstate` | 以现场 `ros2 topic info` 为准 | 官方机身低状态；IMU 在 `imu_state` 字段中 |
| `/sport_odom` | `nav_msgs/msg/Odometry` | Unitree SDK2 `SportModeState` |
| `/sport_imu` | `sensor_msgs/msg/Imu` | Unitree SDK2 `SportModeState` |

检查：

```bash
ros2 topic info /lowstate -v
ros2 topic hz /lowstate
ros2 topic echo /lowstate --once

ros2 topic info /sport_odom -v
ros2 topic hz /sport_odom
ros2 topic echo /sport_odom --once

ros2 topic info /sport_imu -v
ros2 topic hz /sport_imu
ros2 topic echo /sport_imu --once
```

底层 SDK 话题通常是：

```text
rt/lf/sportmodestate
rt/sportmodestate
```

这两个不是 ROS 2 topic，而是 Unitree SDK DDS 通道。新 SLAM 团队如果直接接 Unitree SDK，可以从这些通道取机身状态；如果只接 ROS 2，建议使用 `/sport_odom` 和 `/sport_imu`。

## 7. 推荐给新 SLAM 的输入

建议优先级：

1. 主传感器：当前工程链路使用 `/unitree/slam_lidar/points`；若对接官方 LiDAR service，可先使用当前现场有样本的 `/utlidar/cloud`
2. 辅助 IMU：优先使用 `/lowstate` 中的 `imu_state` 字段，同时可对比 `/sport_imu`
3. 机身里程计：优先评估 `/sport_odom`；当前现场 `/utlidar/robot_odom` 无样本输出，不建议使用
4. `/utlidar/cloud_deskewed` 当前现场有端点但无样本输出，不建议作为必选输入；需要去畸变点云时，应由新 SLAM 团队基于可用点云和机身 IMU/里程计自行处理
5. 不建议把 RF2O 的 `/odom` 作为新 SLAM 的外部里程计输入，除非只是做对比实验

如果新 SLAM 算法需要 2D scan 而不是点云，可以复用当前转换链路：

```bash
ros2 launch go2w_real slam_rf2o.launch.py \
  slam_mode:=mapping \
  use_rviz:=false
```

但对接新 SLAM 时要避免同时让 RF2O 或旧 SLAM 占用 `/odom`、`map -> odom` 等 TF 输出。建议新 SLAM 使用独立输出名调试，例如：

```text
/new_slam/odom
/new_slam/map
TF: map -> odom -> base
```

确认稳定后，再切换到导航栈需要的标准 `/odom` 和 `map -> odom`。

## 8. 最小联调步骤

只启动 XT16 并验证原始数据：

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

另开终端：

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash

ros2 topic list | grep -E 'slam_lidar|utlidar'
ros2 topic hz /unitree/slam_lidar/points
ros2 topic hz /lowstate
ros2 topic echo /lowstate --once
```

启动当前工程的完整参考链路，用于对比：

```bash
ros2 launch go2w_real slam_rf2o.launch.py \
  slam_mode:=mapping \
  use_rviz:=true
```

定位模式加载已有 map：

```bash
ros2 launch go2w_real slam_rf2o.launch.py \
  slam_mode:=localization \
  slam_map_file:=/home/unitree/ros_ws/src/go2w_real/map/zt_0 \
  use_rviz:=true
```

## 9. 建议录包话题

给新 SLAM 团队回放调试时，建议至少录：

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /unitree/slam_lidar/points \
  /lowstate \
  /sport_odom \
  /sport_imu \
  /scan_raw \
  /scan \
  /odom
```

如果只验证新算法对原始数据的处理，最小录包可以是：

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /unitree/slam_lidar/points \
  /lowstate \
  /sport_odom \
  /sport_imu
```

## 10. 对接注意事项

- `/unitree/slam_lidar/points` 是当前最可靠的 XT16 输入源。
- 当前现场 `/utlidar/cloud` 有有效点云，frame_id 是 `utlidar_lidar`；RViz2 显示时需要 Fixed Frame 能 TF 到 `utlidar_lidar`。
- 官方 LiDAR service 文档中提到的 `/utlidar/cloud_deskewed` 当前只有端点但没有样本输出，新 SLAM 团队不要把它作为必选输入。
- XT16 雷达没有 IMU；IMU 数据从 `/lowstate` 的 `imu_state` 字段获取。
- `/lowstate.imu_state` 的时间戳、坐标系和噪声参数仍需新 SLAM 团队自己标定。
- 当前现场 `ros2 topic echo /utlidar/robot_odom` 没有输出，新 SLAM 团队不要依赖这个话题。
- `/utlidar/robot_pose` 在不同机器/固件上可能表现不一致，必须现场用 `ros2 topic hz` 和 `ros2 topic echo` 确认后才能使用。
- `/sport_odom`、`/sport_imu` 来自机身 `SportModeState`，适合作为官方机身状态的 ROS 2 标准消息入口。
- 当前 `/odom` 默认由 RF2O 发布，不是官方里程计。
- 新 SLAM 接入导航前，需要明确谁发布 `map -> odom`，谁发布 `odom -> base`，避免 TF 冲突。
