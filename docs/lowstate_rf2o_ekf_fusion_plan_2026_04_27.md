# /lowstate IMU 与 RF2O 的 EKF 融合方案

日期：2026-04-27  
目标：使用 `/lowstate.imu_state` 辅助 RF2O，输出更稳的局部 `/odom`，降低动态障碍物对 RF2O scan matching 的影响。

## 1. 方案结论

建议采用“三阶段”方案：

1. 新增 `/lowstate -> /lowstate_imu` 转换节点。
2. RF2O 不再直接占用最终 `/odom`，改为发布 `/rf2o/odom`，且关闭 RF2O TF。
3. 使用 `robot_localization/ekf_node` 融合 `/rf2o/odom` 与 `/lowstate_imu`，由 EKF 发布最终 `/odom` 和 `odom -> base` TF。

第一版只融合：

- RF2O 的平面位姿/速度。
- `/lowstate_imu` 的 yaw。
- 谨慎融合 `/lowstate_imu.angular_velocity.z`，必须扣除静止偏置或给较大协方差。

第一版不融合：

- linear acceleration。
- roll/pitch 对二维里程计状态的直接约束。

## 2. 依据

当前静止测试结果：

```text
/lowstate frequency: ~500 Hz
gyro.z static bias mean: -0.009015 rad/s
gyro.z bias repeatability: 0.000018 rad/s between 3 trials
yaw drift over 60 s trials: -0.122, 0.097, 0.316 deg/min
accel norm mean: ~9.44 m/s^2
```

含义：

- `/lowstate.imu_state` 高频且稳定，适合作为 IMU 来源。
- `imu_state.rpy.yaw` 短时间稳定，适合作为 RF2O yaw 的对照和约束。
- `gyroscope.z` 有稳定偏置，不能无处理直接积分。
- 加速度模长低于标准重力，且腿式机器人运动时会有步态冲击，第一版不应融合线加速度。

## 3. 推荐话题架构

```text
/scan
  -> rf2o_laser_odometry
  -> /rf2o/odom

/lowstate
  -> lowstate_imu_bridge
  -> /lowstate_imu

/rf2o/odom + /lowstate_imu
  -> robot_localization/ekf_node
  -> /odom
  -> odom -> base
```

TF 所有权：

| 变换 | 发布者 |
| --- | --- |
| `map -> odom` | `slam_toolbox` |
| `odom -> base` | EKF |
| `base -> lidar/imu` | robot_state_publisher |

关键原则：同一条 TF 只能有一个发布者。启用 EKF 后，RF2O 必须关闭 `publish_tf`，否则会和 EKF 同时发布 `odom -> base`。

## 4. `/lowstate_imu` 转换节点设计

新增节点建议命名：

```text
lowstate_imu_bridge
```

输入：

```text
/lowstate: unitree_go/msg/LowState
```

输出：

```text
/lowstate_imu: sensor_msgs/msg/Imu
```

转换规则：

| `LowState.imu_state` | `sensor_msgs/Imu` |
| --- | --- |
| `quaternion[0..3]` | `orientation` |
| `gyroscope[0..2]` | `angular_velocity` |
| `accelerometer[0..2]` | `linear_acceleration` |
| receive time | `header.stamp` |
| fixed frame | `header.frame_id = imu` |

待确认点：

- Unitree quaternion 数组顺序。当前数据看起来像 `[w, x, y, z]`，ROS `geometry_msgs/Quaternion` 需要 `x, y, z, w`。
- 坐标系是否满足 ROS `base_link` 约定。需要通过 roll/pitch 抬车测试和原地旋转测试确认。
- `/lowstate` 没有标准 header，第一版使用 ROS 接收时间。

建议参数：

```yaml
lowstate_imu_bridge:
  ros__parameters:
    source_topic: /lowstate
    imu_topic: /lowstate_imu
    frame_id: imu
    quaternion_order: wxyz
    gyro_bias:
      x: 0.0
      y: -0.00207
      z: -0.00902
    subtract_gyro_bias: true
    publish_rate_limit_hz: 100.0
```

说明：`/lowstate` 原始频率约 500 Hz，EKF 没必要吃满 500 Hz。第一版可降采样到 100 Hz，减轻 CPU 和队列压力。

## 5. RF2O 调整

当前 `slam_rf2o.launch.py` 中 RF2O 配置为：

```yaml
odom_topic: /odom
publish_tf: true
```

启用 EKF 融合后建议改为：

```yaml
odom_topic: /rf2o/odom
publish_tf: false
base_frame_id: base
odom_frame_id: odom
```

这样 RF2O 只作为 EKF 输入，不再直接发布最终 `/odom` 和 `odom -> base`。

## 6. EKF 第一版配置

建议新增配置文件：

```text
go2w_real/config/odom_fusion_lowstate_rf2o.yaml
```

建议内容：

```yaml
ekf_filter_node_odom:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true
    publish_acceleration: false
    print_diagnostics: true
    transform_time_offset: 0.0
    transform_timeout: 0.2

    map_frame: map
    odom_frame: odom
    base_link_frame: base
    world_frame: odom

    odom0: /rf2o/odom
    odom0_config: [true, true, false,
                   false, false, true,
                   true, true, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    imu0: /lowstate_imu
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 50
    imu0_remove_gravitational_acceleration: false
```

解释：

- `odom0` 使用 RF2O 的 x/y/yaw 以及 vx/vy/yaw_rate。
- `imu0` 只使用 yaw 和 yaw_rate。
- 不使用 IMU 线加速度。
- 若动态测试发现 yaw_rate 偏置扣除后仍不稳，可把 `imu0_config` 中 yaw_rate 关闭，只保留 yaw。

## 7. 协方差建议

转换节点应设置 IMU covariance，避免 EKF 过度信任有偏置的 gyro。

初始建议：

```text
orientation_covariance yaw: 0.03 ~ 0.10
angular_velocity_covariance z: 0.02 ~ 0.10
linear_acceleration_covariance: 大值，且 EKF 不启用
```

如果静止时 EKF `/odom` yaw 仍缓慢漂移：

- 增大 angular_velocity.z covariance。
- 或关闭 yaw_rate，只保留 yaw。
- 或确认 gyro bias 是否在转换节点被正确扣除。

## 8. 验证流程

### 阶段 A：只验证转换节点

```bash
ros2 topic hz /lowstate_imu
ros2 topic echo /lowstate_imu
ros2 run tf2_ros tf2_echo base imu
```

检查：

- `/lowstate_imu` 频率是否稳定在 100 Hz 左右。
- 四元数是否归一化。
- 静止 `angular_velocity.z` 扣偏置后是否接近 0。
- frame_id 是否为 `imu`。

### 阶段 B：RF2O 对比，不启用 EKF

启动 RF2O 输出 `/rf2o/odom`，录包：

```bash
ros2 bag record /lowstate /lowstate_imu /rf2o/odom /tf /tf_static /scan
```

执行低速原地旋转，比较：

- `lowstate_imu yaw`
- `lowstate_imu angular_velocity.z`
- `rf2o/odom yaw`
- `rf2o/odom twist.angular.z`

### 阶段 C：启用 EKF shadow 对比

先不要覆盖生产 `/odom`。建议 EKF 初期输出：

```text
/odom_lowstate_ekf
```

对比稳定后，再切换为最终 `/odom`。

### 阶段 D：替换生产 `/odom`

满足以下条件后再替换：

- 静止 3 分钟 `/odom` 无明显 yaw 漂。
- 原地左右旋转 90 度，EKF yaw 与 IMU/RF2O 方向一致。
- 直线 3 米，EKF yaw 不比 RF2O 更差。
- 动态障碍物干扰时，EKF 比 RF2O 更稳。
- TF 树无重复 `odom -> base` 发布者。

## 9. 风险与处理

| 风险 | 处理 |
| --- | --- |
| gyro.z 有稳定偏置 | 转换节点扣偏置，或关闭 yaw_rate |
| quaternion 顺序不确定 | 用 roll/pitch 抬车测试确认 |
| 坐标系方向不确定 | 用原地左/右转测试确认 z 轴符号 |
| `/lowstate` 无 header | 第一版用接收时间，后续研究 tick 时间映射 |
| RF2O 和 EKF 重复 TF | EKF 启用时 RF2O `publish_tf=false` |
| 动态障碍物导致 RF2O 假位移 | 降低 RF2O yaw 权重，提高 IMU yaw 权重 |

## 10. 推荐落地顺序

1. 实现 `lowstate_imu_bridge.py`，只发布 `/lowstate_imu`。
2. 增加转换节点单元测试：quaternion 顺序、gyro bias、covariance。
3. 新增 `odom_fusion_lowstate_rf2o.yaml`。
4. 修改 launch：融合模式下 RF2O 输出 `/rf2o/odom` 且 `publish_tf=false`。
5. 先 shadow 输出 `/odom_lowstate_ekf` 做 bag 对比。
6. 实机验证通过后，再让 EKF 接管 `/odom`。

推荐第一版不要直接替换当前导航链路。先 shadow 对比，等 yaw 方向、偏置扣除、TF 所有权都确认后再接入 Nav2。

## 11. 更新：优先评估 /utlidar/imu

2026-04-27 追加静止和小角度转向测试后，IMU 输入优先级需要调整：

- `/utlidar/imu` 静止 gyro.z 零偏约 `0.00061 rad/s`，明显优于 `/lowstate.imu_state` 的约 `-0.00902 rad/s`。
- `/utlidar/imu` 有标准 ROS `header.stamp` 和 `frame_id=utlidar_imu`。
- 但小角度转向显示：正向 `cmd_vel.angular.z` 时，`/utlidar/imu.angular_velocity.z` 主要为负；反向时主要为正。
- 当前 URDF 没有显式 `utlidar_imu` frame，只能从 `lidar`/`imu` frame 做估计，不能直接解决轴向。

因此，新的建议是：

1. 优先评估 `/utlidar/imu` 作为 yaw_rate 来源。
2. 接 EKF 前先做 `utlidar_imu -> base` 轴向修正。
3. 第一版经验映射可先按 `base_yaw_rate ~= -utlidar_imu.angular_velocity.z` 做 shadow 验证。
4. 不要把原始 `/utlidar/imu` 直接接入 EKF，否则 yaw 方向可能反。
5. `/lowstate.imu_state` 保留作对照和机身姿态参考。
