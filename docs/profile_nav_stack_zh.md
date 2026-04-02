# profile_nav_stack 使用说明

`profile_nav_stack.py` 用来统计 GO2W 实机 SLAM + Nav2 相关节点的 CPU 占用，并把从启动 profiler 到退出 profiler 这段时间内的所有采样结果落盘。

这次已经补上了 `./xt16_driver` 的 CPU 统计。

## 能记录什么

- `xt16_driver`
- `go2w_bridge`
- `pointcloud_relay`
- `pointcloud_to_laserscan`
- `scan_to_scan_filter_chain`
- `rf2o_laser_odometry`
- `wait_for_odom_to_lidar_tf`
- `wait_for_map_to_lidar_tf`
- `slam_toolbox`
- `controller_server`
- `planner_server`
- `recoveries_server`
- `bt_navigator`
- `lifecycle_manager_navigation`
- `robot_state_publisher`
- `base_to_base_footprint`
- `lidar_to_rslidar`
- `rviz2`
- `navigate_to_waypoint`
- `navigation_executor`

## 运行方式

建议先启动雷达，再启动 SLAM / Nav2，最后单独开一个终端运行 profiler。

### 1. 启动 XT16 雷达

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

### 2. source ROS 环境

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
```

### 3. 启动 SLAM + Nav2

```bash
ros2 launch go2w_real slam_rf2o.launch.py
```

### 4. 启动 CPU profiler

```bash
ros2 run go2w_real profile_nav_stack.py
```

常用参数：

```bash
ros2 run go2w_real profile_nav_stack.py \
  --interval 1.0 \
  --output-dir ~/ros_ws/profile_logs/run_01 \
  --show-missing
```

- `--interval`：采样周期，单位秒
- `--samples`：采样次数，默认 `0`，表示一直采到你按 `Ctrl+C`
- `--output-dir`：输出目录；不传时默认写到当前目录下的 `./profile_nav_stack_logs/<timestamp>/`
- `--show-missing`：实时表格里也显示当前没跑起来的节点

## 输出文件

脚本退出后，会在输出目录生成 3 个文件：

- `samples.csv`
- `summary.csv`
- `summary.md`

### samples.csv

这是完整时序采样记录。

- 每一行代表一次采样
- 每一列代表一个节点
- 从 profiler 启动到退出之间的所有 CPU 数据都会保存在这里

字段说明：

- `sample_index`：第几个样本
- `timestamp`：采样时间
- `elapsed_sec`：从启动 profiler 开始累计的秒数
- `stack_total_cpu_percent`：所有被监控节点 CPU 占用之和
- 其余列：对应节点当次采样的 CPU%

### summary.csv

这是适合做表格分析的汇总结果，包含每个节点的平均值、最大值和被检测到的样本数。

### summary.md

这是便于直接阅读的 Markdown 报告，里面会列出每个节点的平均 CPU 占用。

## 平均值怎么算

汇总里有两类平均值：

- `Avg CPU% (whole run)`：按整个监控窗口求平均
- `Avg CPU% (when running)`：只在进程实际存在时求平均

说明：

- 如果某个节点在 profiler 启动后还没起来，或者中途退出了，那么它在不存在的那些采样点会按 `0.0` 计入 `whole run`
- 如果你只想看节点真正运行期间的平均负载，参考 `when running`

## 退出时终端会看到什么

按 `Ctrl+C` 后，终端会打印一张最终汇总表，里面至少包含：

- 每个节点在整个监控窗口内的平均 CPU%
- 每个节点运行期间的平均 CPU%
- 每个节点的最大 CPU%
- 每个节点出现了多少个样本

## 一个典型流程

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
ros2 launch go2w_real slam_rf2o.launch.py
```

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
ros2 run go2w_real profile_nav_stack.py --output-dir ~/ros_ws/profile_logs/nav_stack_run
```

停止后直接看：

- `~/ros_ws/profile_logs/nav_stack_run/samples.csv`
- `~/ros_ws/profile_logs/nav_stack_run/summary.csv`
- `~/ros_ws/profile_logs/nav_stack_run/summary.md`
