# /lowstate.imu_state 静止测试报告

测试时间：2026-04-27 14:22 CST  
测试地点：实机 ROS 2 环境，`/home/unitree/ros_ws/src`  
测试对象：`/lowstate` 中的 `imu_state` 字段  
测试用例：用例一，静止零偏与稳定性测试

## 1. 测试目的

本次测试用于判断 `/lowstate.imu_state` 在机器人静止时是否足够稳定，是否具备后续和 RF2O `/odom` 做融合的基础条件。

重点检查：

- `/lowstate` 是否在线，消息类型是否正确。
- `imu_state` 的姿态、角速度、加速度字段是否有有效输出。
- 静止 60 秒内姿态是否稳定。
- 静止 60 秒内 gyro 零偏是否可接受。
- 加速度模长是否接近重力加速度。
- 采样频率和时间间隔是否稳定。

## 2. 环境信息

ROS 版本：

```bash
ROS_DISTRO=foxy
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`/lowstate` 话题信息：

```text
Type: unitree_go/msg/LowState
Publisher count: 1
Reliability: RELIABLE
Durability: VOLATILE
```

`imu_state` 字段结构：

```text
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature
```

## 3. 测试方法

机器人保持静止，不发送 `/cmd_vel`，连续订阅 `/lowstate` 60 秒。

采样脚本使用 Reliable QoS 订阅 `/lowstate`，以本机接收时间计算频率和间隔抖动，并统计：

- `imu_state.quaternion`
- `imu_state.gyroscope`
- `imu_state.accelerometer`
- `imu_state.rpy`
- `imu_state.temperature`
- `LowState.tick`

## 4. 采样结果

采样总时长：

```text
60.0013 s
```

样本数：

```text
30000
```

等效频率：

```text
500.001 Hz
```

接收间隔：

| 指标 | 数值 |
| --- | ---: |
| mean | 0.002000 s |
| std | 0.000506 s |
| min | 0.000817 s |
| max | 0.056564 s |

说明：平均频率稳定在约 500 Hz，但接收时间存在一次较大的调度间隔峰值。该峰值更可能来自 ROS/Python 订阅端调度，不一定代表底层 IMU 真实丢样；后续如要精确评估延迟，应优先使用消息源内部时间戳或 `tick`。

## 5. 姿态稳定性

RPY 统计，单位为度：

| 轴 | mean | std | min | max |
| --- | ---: | ---: | ---: | ---: |
| roll | -0.6490 | 0.0348 | -0.7310 | -0.5718 |
| pitch | 1.8860 | 0.0517 | 1.7758 | 1.9846 |
| yaw | 7.8369 | 0.1111 | 7.6403 | 8.0572 |

首末 yaw 变化折算：

```text
yaw drift = -0.360 deg/min
```

四元数模长：

| 指标 | 数值 |
| --- | ---: |
| mean | 1.000000108 |
| std | 0.000000049 |
| min | 0.999999950 |
| max | 1.000000268 |

结论：静止时姿态输出连续且稳定，四元数归一化质量很好。roll/pitch 抖动小，yaw 在 60 秒内只有轻微变化。

## 6. 角速度零偏

Gyroscope 统计，单位为 rad/s：

| 轴 | mean | std | min | max |
| --- | ---: | ---: | ---: | ---: |
| x | -0.000033 | 0.006704 | -0.026632 | 0.029827 |
| y | -0.002061 | 0.013078 | -0.043676 | 0.039415 |
| z | -0.008890 | 0.006416 | -0.034088 | 0.015979 |

Gyroscope 模长：

| 指标 | 数值 |
| --- | ---: |
| mean | 0.017141 rad/s |
| std | 0.006826 rad/s |
| max | 0.048828 rad/s |

结论：x 轴均值接近 0，y 轴有轻微偏置，z 轴静止均值约 `-0.00889 rad/s`。如果直接把 `gyroscope.z` 当作 yaw rate 融入 EKF，这个偏置可能造成累计航向误差；后续融合前建议做静止零偏扣除、提高 yaw rate 协方差，或优先测试融合 orientation yaw 而不是无校准 yaw rate。

## 7. 加速度稳定性

Accelerometer 统计，单位为 m/s^2：

| 轴 | mean | std | min | max |
| --- | ---: | ---: | ---: | ---: |
| x | -0.697929 | 0.026741 | -0.810437 | -0.578200 |
| y | 0.010580 | 0.026935 | -0.120907 | 0.153229 |
| z | 9.413610 | 0.042924 | 9.190144 | 9.609129 |

加速度模长：

| 指标 | 数值 |
| --- | ---: |
| mean | 9.439530 m/s^2 |
| std | 0.042749 m/s^2 |
| min | 9.208314 m/s^2 |
| max | 9.632921 m/s^2 |

结论：加速度方向和静止状态下的重力主分量一致，稳定性较好。模长约 `9.44 m/s^2`，低于标准重力 `9.81 m/s^2`，可能与传感器标定、安装姿态、Unitree 内部滤波或量纲处理有关。当前不建议第一版 EKF 融合 linear acceleration。

## 8. 温度

```text
temperature = 79
```

60 秒内温度字段保持不变。该字段单位以 Unitree 消息定义为准，本报告只记录原始值。

## 9. 初步结论

本次静止用例结果为：`/lowstate.imu_state` 基础可用，姿态输出稳定，话题频率高且连续，适合作为后续 RF2O 对比和融合实验的候选 IMU 来源。

但还不能直接进入最终融合配置，原因是：

- `gyroscope.z` 静止均值约 `-0.00889 rad/s`，对 yaw rate 融合偏大。
- 加速度模长约 `9.44 m/s^2`，不建议直接融合线加速度。
- `/lowstate` 消息本身没有标准 ROS `header.stamp`，需要转换节点明确时间戳策略。

建议下一步：

1. 继续执行用例三“原地旋转 yaw 测试”，验证 yaw、gyro.z、RF2O yaw 三者方向和量级是否一致。
2. 做 3 到 5 组静止 60 秒重复测试，确认 gyro.z 偏置是否稳定。
3. 若偏置稳定，在 `/lowstate -> sensor_msgs/Imu` 转换节点中提供静止零偏参数。
4. 第一版 EKF 不融合 linear acceleration。
5. 第一版 EKF 可先对比两种方案：
   - 只融合 `/lowstate_imu.orientation.z/yaw`
   - 融合 `/lowstate_imu.orientation.yaw + angular_velocity.z`，但提高 yaw rate 协方差

## 10. 判定

用例一判定：条件通过。

含义：`/lowstate.imu_state` 在静止场景下具备继续测试价值。  
限制：gyro.z 零偏需要在动态测试和融合前进一步处理。
