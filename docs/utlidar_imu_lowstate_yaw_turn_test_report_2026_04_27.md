# /utlidar/imu 与 /lowstate.imu_state 对比及小角度转向测试报告

测试时间：2026-04-27 15:11 CST  
测试对象：`/utlidar/imu`、`/lowstate.imu_state`、`/lf/sportmodestate`  
测试目的：判断 `/utlidar/imu` 和 `/lowstate.imu_state` 哪个更适合作为 RF2O/EKF 融合的 IMU 输入，并在不超过 90 度的限制下验证 `/utlidar/imu` z 轴方向。

## 1. 结论摘要

静止稳定性：

- `/utlidar/imu` 的 gyro.z 静止零偏明显小于 `/lowstate.imu_state`。
- `/utlidar/imu` 有标准 ROS `header.stamp` 和 `frame_id=utlidar_imu`。
- `/lowstate.imu_state` 频率更高，但没有 header，gyro.z 静止偏置约 `-0.009 rad/s`。

动态小角度转向：

- 正向 `cmd_vel.angular.z` 时，`/lowstate.imu_state` yaw 正向变化。
- 同一阶段 `/utlidar/imu.angular_velocity.z` 主要为负。
- 反向复位时，`/lowstate.imu_state` yaw 负向变化，而 `/utlidar/imu.angular_velocity.z` 主要为正。

因此：

```text
/utlidar/imu 静止质量更好，但 utlidar_imu 的 z 轴方向与 base yaw 方向很可能相反。
```

接 EKF 前不能直接把 `/utlidar/imu.angular_velocity.z` 当作 `base` yaw rate 使用，必须先做 `utlidar_imu -> base` 的外参/轴向修正。

## 2. URDF 外参估计

当前 URDF 中没有显式 `utlidar_imu` frame。

已有近似传感器 frame：

```xml
<joint name="imu_joint" type="fixed">
  <origin xyz="-0.02557 0 0.04232" rpy="0 0 0" />
  <parent link="body" />
  <child link="imu" />
</joint>

<joint name="lidar_joint" type="fixed">
  <origin xyz="0.28945 0 -0.046825" rpy="0 0 0" />
  <parent link="body" />
  <child link="lidar" />
</joint>
```

`base -> body` 无额外旋转。  
因此可先把 `/utlidar/imu` 近似认为在 `lidar` 附近，但 URDF 目前不能直接给出 `utlidar_imu` 的真实轴向。

本次测试前未发现 `/tf_static` 发布 `utlidar_imu`：

```text
WARNING: topic [/tf_static] does not appear to be published yet
```

## 3. 60 秒静止对比

采样期间不发布任何运动命令。

| 指标 | `/utlidar/imu` | `/lowstate.imu_state` |
| --- | ---: | ---: |
| 样本数 | 14988 | 30001 |
| 频率 | 249.800 Hz | 500.023 Hz |
| ROS header | 有 | 无 |
| frame | `utlidar_imu` | 无标准 frame |
| gyro.z mean | `0.000610 rad/s` | `-0.009015 rad/s` |
| gyro.z std | `0.003754 rad/s` | `0.006472 rad/s` |
| yaw std | `0.0165 deg` | `0.1252 deg` |
| yaw drift | `-0.0022 deg/min` | `-0.4822 deg/min` |
| accel norm mean | `10.1261 m/s^2` | `9.4601 m/s^2` |

`/utlidar/imu` 的 covariance 当前全为 0：

```text
orientation_covariance: all zero
angular_velocity_covariance: all zero
linear_acceleration_covariance: all zero
```

这表示 covariance 未标定，接入 EKF 时需要在转换层或滤波配置中补合理协方差。

## 4. 小角度正反向转向测试

用户约束：不要超过 90 度。

实际命令：

| 阶段 | 命令 | 持续时间 | 理论角度 |
| --- | ---: | ---: | ---: |
| ccw_positive_cmd | `angular.z=+0.20 rad/s` | 2.0 s | +22.9 deg |
| settle_after_ccw | `angular.z=0` | 1.0 s | 0 |
| cw_negative_cmd_reset | `angular.z=-0.20 rad/s` | 2.0 s | -22.9 deg |
| final_stop | `angular.z=0` | 3.0 s | 0 |

脚本设置了 75 度强制停止保护。测试未触发 abort。

测试后连续发布零速度 3 秒，并关闭 `go2w_bridge.py`。关闭后 ROS 图中无 `/cmd_vel`、`/sport_odom`、`/sport_imu`。

## 5. 转向观测结果

### 5.1 正向 angular.z

| 指标 | 数值 |
| --- | ---: |
| `/utlidar/imu.angular_velocity.z` mean | `-0.0592 rad/s` |
| `/utlidar/imu` z 积分 | `-7.04 deg` |
| `/lowstate.imu_state.gyroscope.z` mean | `+0.1195 rad/s` |
| `/lowstate` gyro.z 积分 | `+17.51 deg` |
| `/lowstate` yaw delta | `+18.22 deg` |
| `/lf/sportmodestate` yaw delta | `+17.54 deg` |

### 5.2 反向 angular.z 复位

| 指标 | 数值 |
| --- | ---: |
| `/utlidar/imu.angular_velocity.z` mean | `+0.0427 rad/s` |
| `/utlidar/imu` z 积分 | `+4.97 deg` |
| `/lowstate.imu_state.gyroscope.z` mean | `-0.0782 rad/s` |
| `/lowstate` gyro.z 积分 | `-10.00 deg` |
| `/lowstate` yaw delta | `-11.27 deg` |
| `/lf/sportmodestate` yaw delta | `-11.91 deg` |

## 6. 解释

`/lowstate.imu_state` 和 `/lf/sportmodestate` 的 yaw 方向与命令方向一致：

- 正向 `angular.z` -> yaw 增加。
- 反向 `angular.z` -> yaw 减少。

`/utlidar/imu.angular_velocity.z` 与上述方向相反：

- 正向 `angular.z` -> `/utlidar/imu.z` 主要为负。
- 反向 `angular.z` -> `/utlidar/imu.z` 主要为正。

这说明 `/utlidar/imu` 的 z 轴方向与机器人 `base` yaw 方向很可能相反。由于 URDF 没有 `utlidar_imu` frame，本次不能仅靠 URDF 解决轴向问题，需要增加一个估计静态变换或在 IMU 转换节点里做轴向映射。

## 7. EKF 建议

优先级建议：

1. 位移仍由 RF2O 提供。
2. yaw/yaw_rate 优先考虑 `/utlidar/imu`，因为静止噪声和零偏更好。
3. 但 `/utlidar/imu` 必须先完成轴向修正，尤其是 yaw_rate z 符号。
4. `/lowstate.imu_state` 可作为机身姿态对照和验证源。

第一版可采用保守转换：

```text
base_yaw_rate ≈ -utlidar_imu.angular_velocity.z
```

但这只是基于本次小角度测试的经验映射。正式接入前还应做：

- 左右各 2 到 3 次小角度复测。
- 与 RF2O `/odom` yaw 做同步对比。
- 确认 `utlidar_imu -> base` 的完整 roll/pitch/yaw 外参。

## 8. 风险

- 本次动态测试主要用于符号判断，不是完整标定。
- `/utlidar/imu` 姿态四元数在静止时 roll/pitch 与机身姿态不一致，说明 frame 轴向确实不是 `base` 轴向。
- `/utlidar/imu` covariance 全为 0，必须手动设置合理协方差。
- 如果直接把原始 `/utlidar/imu` 接入 EKF，yaw 方向可能被反向约束，导致 `/odom` 更差。
