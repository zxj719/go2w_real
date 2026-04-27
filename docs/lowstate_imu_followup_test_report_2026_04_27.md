# /lowstate.imu_state 后续测试报告

测试时间：2026-04-27 14:27 CST  
测试对象：`/lowstate.imu_state`  
测试目标：执行上一份静止报告中的后续计划，确认 gyro.z 偏置稳定性，并检查是否具备 RF2O 动态对比测试条件。

## 1. 执行摘要

本次继续执行了 3 组静止 60 秒复测，每组均成功采集 `/lowstate.imu_state`。

结论：

- `/lowstate` 持续在线，`imu_state` 输出稳定。
- 三组采样频率均约 500 Hz。
- `gyroscope.z` 静止偏置高度稳定，平均约 `-0.009015 rad/s`。
- 该偏置约等于 `-0.5165 deg/s`，若不扣除，理论累计量约 `-31.0 deg/min`。
- 当前系统未发现 `/odom`、`/cmd_vel`、RF2O、`go2w_bridge`、EKF 等运行链路，因此未执行原地旋转和 RF2O 对比测试。

本次没有向机器人发布运动指令。

## 2. 当前 ROS 链路检查

当前可见关键话题：

```text
/lowstate
```

当前未发现：

```text
/odom
/cmd_vel
/sport_imu
/sport_odom
rf2o_laser_odometry
go2w_bridge
ekf_node
```

检查结果：

```text
ros2 topic info /odom    -> Unknown topic '/odom'
ros2 topic info /cmd_vel -> Unknown topic '/cmd_vel'
```

因此，本轮只执行安全的静止复测；原地旋转测试需要在 `slam_rf2o.launch.py` 或对应导航/桥接链路启动后再执行。

## 3. 静止复测方法

机器人保持静止，不发送 `/cmd_vel`。

连续执行 3 组采样：

- 每组 60 秒。
- 组间间隔 3 秒。
- 使用 Reliable QoS 订阅 `/lowstate`。
- 使用本机接收时间统计频率和间隔。

## 4. 三组复测结果

### 4.1 采样频率

| 轮次 | 样本数 | 等效频率 |
| --- | ---: | ---: |
| 1 | 30000 | 500.006 Hz |
| 2 | 30012 | 500.184 Hz |
| 3 | 30012 | 500.185 Hz |

结论：频率稳定，满足后续转换为标准 IMU topic 的实时性要求。

### 4.2 gyroscope.z 静止偏置

| 轮次 | gyro.z mean | gyro.z std | min | max |
| --- | ---: | ---: | ---: | ---: |
| 1 | -0.00903685 rad/s | 0.00646959 | -0.04048004 | 0.01704423 |
| 2 | -0.00899185 rad/s | 0.00641851 | -0.03728425 | 0.01704423 |
| 3 | -0.00901698 rad/s | 0.00642366 | -0.03834952 | 0.01917476 |

三组均值汇总：

```text
gyro_z_bias_mean = -0.00901523 rad/s
gyro_z_bias_std_between_trials = 0.00001841 rad/s
gyro_z_bias = -0.5165 deg/s
gyro_z_bias = -30.99 deg/min
```

结论：`gyro.z` 静止偏置不是偶发噪声，而是稳定偏置。后续如果融合 yaw rate，应在 `/lowstate -> sensor_msgs/Imu` 转换层扣除该偏置，或在 EKF 中给 yaw rate 较大的协方差。

### 4.3 yaw 静止漂移

| 轮次 | yaw drift |
| --- | ---: |
| 1 | -0.1224 deg/min |
| 2 | 0.0973 deg/min |
| 3 | 0.3161 deg/min |

结论：`rpy.yaw` 本身的短时间漂移明显小于直接积分未扣偏置的 `gyro.z`。这说明 `imu_state.rpy` 很可能来自 Unitree 内部姿态估计，而不是简单积分原始 gyro。

### 4.4 加速度模长

| 轮次 | accel norm mean |
| --- | ---: |
| 1 | 9.4421 m/s^2 |
| 2 | 9.4417 m/s^2 |
| 3 | 9.4444 m/s^2 |

结论：加速度模长重复性很好，但均值仍低于标准重力 `9.81 m/s^2`。第一版 EKF 仍不建议融合 linear acceleration。

### 4.5 姿态稳定性

| 轮次 | roll mean | pitch mean | yaw mean |
| --- | ---: | ---: | ---: |
| 1 | -0.5549 deg | 1.8325 deg | 7.3234 deg |
| 2 | -0.5806 deg | 1.9054 deg | 7.2825 deg |
| 3 | -0.5937 deg | 1.8740 deg | 7.3895 deg |

结论：roll、pitch、yaw 在静止状态下重复性较好，姿态输出适合作为后续 yaw 对比测试的观测源。

## 5. 动态测试状态

上一份报告建议继续做“原地旋转 yaw 测试”。本次未执行，原因是当前系统没有发现可用控制和 RF2O 对比链路：

- 没有 `/cmd_vel`。
- 没有 `/odom`。
- 没有 RF2O 节点进程。
- 没有 `go2w_bridge` 节点进程。

为避免不可控的实机运动，本次未尝试直接发布运动指令。

动态测试复跑条件：

1. 启动 `go2w_bridge` 或完整 `slam_rf2o.launch.py` 链路。
2. 确认 `/cmd_vel` 有订阅端。
3. 确认 `/odom` 或后续 `/rf2o/odom` 有发布端。
4. 确认现场具备原地低速旋转安全空间。
5. 再执行低速原地转向 bag 记录。

建议动态测试命令：

```bash
ros2 bag record \
  /lowstate \
  /odom \
  /tf \
  /tf_static \
  /scan \
  -o lowstate_rf2o_yaw_turn_test
```

低速原地旋转建议由人工现场确认安全后执行：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.20}}" -r 10
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

## 6. 判定

静止复测判定：通过。

`/lowstate.imu_state` 的姿态和加速度输出稳定，`gyro.z` 偏置稳定可标定。  
下一步可以进入 `/lowstate -> sensor_msgs/Imu` 转换节点设计，以及 RF2O/IMU 融合方案设计。
