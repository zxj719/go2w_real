# /lf/sportmodestate 里程计静止与短直线测试报告

测试时间：2026-04-27 14:51 CST  
测试对象：`/lf/sportmodestate`  
测试目的：验证 `/lf/sportmodestate.position`、`velocity`、`yaw_speed` 是否适合作为机身里程计来源，并执行不超过 0.5 m 的短直线往返测试。

## 1. 安全约束

用户约束：直线测试不超过 0.5 m，测试后复位。

实际执行：

- 先做 20 秒静止基线。
- 启动 `go2w_bridge.py`，只用于临时提供 `/cmd_vel` 控制入口和 `/sport_odom` 输出。
- 前进命令：`vx = 0.05 m/s`，持续 `2.0 s`。
- 理论单程位移：`0.10 m`。
- 停止后反向复位命令：`vx = -0.05 m/s`，持续 `2.0 s`。
- 最后连续发布零速度，并关闭 `go2w_bridge.py`。

本次理论单程位移远低于 0.5 m。  
测试结束后，`go2w_bridge.py` 已关闭，ROS 图中只剩 `/lf/sportmodestate`，未保留 `/cmd_vel` 控制链路。

## 2. 话题与消息

`/lf/sportmodestate` 当前在线：

```text
Type: unitree_go/msg/SportModeState
Publisher count: 1
```

里程计相关字段：

```text
position: float32[3]
velocity: float32[3]
yaw_speed: float32
imu_state: IMUState
```

注意：样本中的 `stamp.sec` 和 `stamp.nanosec` 均为 `0`，不能直接作为 ROS 里程计时间戳。

## 3. 20 秒静止基线

机器人未接收 `/cmd_vel` 运动命令时，采集 `/lf/sportmodestate` 20 秒。

采样结果：

| 指标 | 数值 |
| --- | ---: |
| 样本数 | 9975 |
| 等效频率 | 498.718 Hz |
| mode | 1 |
| position xy 漂移 | 0.1534 m |
| velocity xy 均值 | 0.0379 m/s |
| yaw_speed 均值 | 0.0 rad/s |

position 起点：

```text
[-24.2804, 17.3961, 0.4211]
```

position 终点：

```text
[-24.4074, 17.4823, 0.4215]
```

20 秒静止时 position 增量：

```text
[-0.1269, 0.0862, 0.0004]
```

结论：`/lf/sportmodestate.position` 在静止时存在明显漂移，`velocity.xy` 也长期非零。因此该 position/velocity 不能直接当静止真值或硬距离限幅依据。

## 4. 短直线往返测试

命令规划：

| 阶段 | 命令 | 持续时间 | 理论位移 |
| --- | ---: | ---: | ---: |
| forward | `vx=+0.05 m/s` | 2.0 s | +0.10 m |
| stop | `vx=0.0 m/s` | 1.0 s | 0 |
| reverse_reset | `vx=-0.05 m/s` | 2.0 s | -0.10 m |
| final_stop | `vx=0.0 m/s` | 3.0 s | 0 |

测试脚本还设置了观测位移保护：

```text
max_observed_delta_abort_m = 0.45
```

测试未触发 abort。

## 5. 短直线测试观测结果

命令执行窗口内：

| 指标 | 数值 |
| --- | ---: |
| 样本数 | 227 |
| 起终点 position xy 差 | 0.0871 m |
| 最大 position xy 差 | 0.0871 m |
| velocity xy 均值 | 0.0357 m/s |
| velocity x 均值 | -0.0326 m/s |
| velocity y 均值 | 0.0141 m/s |
| yaw_speed 均值 | 0.0 rad/s |
| mode | 1, 3 |

起点：

```text
[-24.7554, 17.7415, 0.4222]
```

终点：

```text
[-24.8160, 17.8040, 0.4215]
```

起终点增量：

```text
[-0.0606, 0.0625, -0.0007]
```

观察到的关键现象：

- 发布 `vx=+0.05 m/s` 时，`/lf/sportmodestate.velocity.x` 没有呈现明确正向响应。
- 发布反向 `vx=-0.05 m/s` 时，position 也没有按反向命令回到起点，而是继续沿静止漂移趋势变化。
- `yaw_speed` 始终为 0，说明本次短直线没有明显转向。

## 6. 复位后静止确认

反向复位命令结束后，额外连续发布零速度并做 10 秒静止检查。

结果：

| 指标 | 数值 |
| --- | ---: |
| 样本数 | 197 |
| position xy 漂移 | 0.0545 m |
| velocity xy 均值 | 0.0332 m/s |
| velocity x 均值 | -0.0310 m/s |
| velocity y 均值 | 0.0117 m/s |
| yaw_speed 均值 | 0.0 rad/s |
| mode | 1 |

这说明复位后 `/lf/sportmodestate.position` 仍继续漂移，且 velocity 仍非零。该现象与测试前静止基线一致。

## 7. 结论

`/lf/sportmodestate` 确实包含里程计相关字段，并且频率很高，约 500 Hz。

但本次实测显示：

- 静止时 position 20 秒漂移约 0.153 m。
- 静止时 velocity.xy 均值约 0.038 m/s。
- 短直线正反向命令下，position/velocity 没有呈现清晰的命令方向响应。
- `stamp` 为 0，转 ROS 标准里程计时必须使用接收时间。
- 该数据源不能直接作为高可信 `/odom` 真值，也不适合直接用作短距离闭环复位依据。

因此，`/lf/sportmodestate` 可作为候选机身状态来源，但接入 EKF 时应非常保守：

1. 不建议直接融合 position。
2. 不建议直接高权重融合 velocity。
3. 可以继续评估 yaw、yaw_speed、IMU 姿态部分。
4. 若使用 position/velocity，应先做长时间静止漂移建模，并设置较大 covariance。
5. RF2O 仍应保留为主要局部位移来源，`/lf/sportmodestate` 更适合先作为对照数据或低权重辅助项。

## 8. 后续建议

下一步建议不要继续加大直线位移，而是先做两项验证：

1. 外部尺量或视觉标记：实测 0.1 m 低速命令是否真的产生物理位移。
2. 对比 RF2O `/odom`：同一段短直线下，比较 RF2O 与 `/lf/sportmodestate.position` 的方向和尺度。

只有当 `/lf/sportmodestate` 的方向、尺度和静止漂移原因明确后，再考虑把它作为 EKF 的 odom 输入。

## 9. 追加测试：提高速度到 0.15 m/s

用户反馈上一轮 `0.05 m/s` 速度下机器人基本没动，因此追加一轮更高速度的短直线往返测试。

安全约束保持不变：

- 单程不超过 0.5 m。
- 测试后反向复位。
- 最后连续发布零速度并关闭 `go2w_bridge.py`。

命令规划：

| 阶段 | 命令 | 持续时间 | 理论位移 |
| --- | ---: | ---: | ---: |
| forward_v015 | `vx=+0.15 m/s` | 2.0 s | +0.30 m |
| stop | `vx=0.0 m/s` | 1.0 s | 0 |
| reverse_reset_v015 | `vx=-0.15 m/s` | 2.0 s | -0.30 m |
| final_stop | `vx=0.0 m/s` | 3.0 s | 0 |

理论单程位移 `0.30 m`，低于用户设定的 `0.5 m` 上限。

### 9.1 测试结果

| 指标 | 数值 |
| --- | ---: |
| 样本数 | 227 |
| 起终点 position xy 差 | 0.0816 m |
| 最大 position xy 差 | 0.2236 m |
| velocity xy 均值 | 0.0628 m/s |
| velocity x 最小值 | -0.1543 m/s |
| velocity x 最大值 | 0.0881 m/s |
| yaw_speed 均值 | 0.0 rad/s |
| mode | 1, 3 |

起点：

```text
[-25.8609, 18.5186, 0.4224]
```

终点：

```text
[-25.9201, 18.5746, 0.4200]
```

起终点增量：

```text
[-0.0593, 0.0560, -0.0024]
```

阶段观测：

| 阶段 | 结束时 position | 结束时 velocity |
| --- | --- | --- |
| forward_v015 | `[-25.6897, 18.5584, 0.4198]` | `[0.0826, 0.0221, -0.0324]` |
| settle_after_forward | `[-25.6501, 18.5762, 0.4224]` | `[-0.0347, 0.0176, -0.0345]` |
| reverse_reset_v015 | `[-25.8439, 18.5628, 0.4210]` | `[-0.1501, 0.0082, -0.0347]` |
| final_stop | `[-25.9076, 18.5672, 0.4199]` | `[-0.0341, 0.0107, -0.0335]` |

结论：

- 相比 `0.05 m/s`，`0.15 m/s` 下 `/lf/sportmodestate.velocity.x` 出现了明确的正反向响应。
- 前进阶段 velocity.x 到约 `+0.083 m/s`。
- 反向复位阶段 velocity.x 到约 `-0.150 m/s`。
- 最大观测 position xy 差约 `0.224 m`，没有超过 `0.5 m`。
- `yaw_speed` 仍为 0，说明本次主要是直线运动，没有明显原地转向。

### 9.2 复位后静止确认

反向复位后，额外持续发布零速度并做静止检查。

| 指标 | 数值 |
| --- | ---: |
| 检查时长 | 12.43 s |
| 样本数 | 236 |
| position xy 漂移 | 0.0830 m |
| velocity xy 均值 | 0.0349 m/s |
| velocity x 均值 | -0.0328 m/s |
| velocity y 均值 | 0.0119 m/s |
| yaw_speed 均值 | 0.0 rad/s |
| mode | 1 |

复位动作已执行，`go2w_bridge.py` 已关闭。关闭后 ROS 图中只剩 `/lf/sportmodestate`，没有保留 `/cmd_vel`、`/sport_odom`、`/sport_imu`。

### 9.3 更新结论

`0.15 m/s` 是更合适的短直线测试速度：它能激发 `/lf/sportmodestate.velocity.x` 的正反向响应，同时理论单程仍低于 0.5 m。

不过，复位后静止检查仍看到 `/lf/sportmodestate.position` 持续漂移、`velocity.xy` 非零。因此最终结论保持谨慎：

- `/lf/sportmodestate.velocity.x` 可以反映较明显的前后运动趋势。
- `/lf/sportmodestate.position` 静止漂移仍然明显，不适合直接作为 EKF 高权重 position 输入。
- 若纳入 EKF，建议先只把 `/lf/sportmodestate` 作为低权重速度辅助或 shadow 对照，不要直接替代 RF2O。

## 10. 追加测试：干净静止 60 秒复测

用户提示上一轮静止漂移可能受到遥控器操作影响，因此追加一次更干净的静止复测。

复测前状态：

- 未启动 `go2w_bridge.py`。
- ROS 图中没有 `/cmd_vel`。
- ROS 图中没有 `/sport_odom`、`/sport_imu`。
- 只订阅 `/lf/sportmodestate`，不发布任何运动命令。
- 采样期间要求遥控器不动、机器人保持静止。

复测时间：2026-04-27 14:59 CST。

### 10.1 总体结果

| 指标 | 数值 |
| --- | ---: |
| 采样时长 | 59.998 s |
| 样本数 | 29949 |
| 等效频率 | 499.153 Hz |
| mode | 1 |
| gait_type | 0 |
| stamp | `[0, 0] -> [0, 0]` |
| position xy 漂移 | 0.4469 m |
| 漂移速率 | 0.4469 m/min |
| velocity xy 均值 | 0.0363 m/s |
| velocity x 均值 | -0.0339 m/s |
| velocity y 均值 | 0.0128 m/s |
| yaw_speed 均值 | 0.0 rad/s |

起点：

```text
[-27.2162, 19.1743, 0.4206]
```

终点：

```text
[-27.6287, 19.3464, 0.4211]
```

60 秒 position 增量：

```text
[-0.4124, 0.1721, 0.0006]
```

### 10.2 10 秒分段漂移

| 分段 | position xy 漂移 | 漂移速率 | velocity xy 均值 |
| --- | ---: | ---: | ---: |
| 0-10 s | 0.0746 m | 0.4476 m/min | 0.0364 m/s |
| 10-20 s | 0.0740 m | 0.4438 m/min | 0.0363 m/s |
| 20-30 s | 0.0746 m | 0.4474 m/min | 0.0369 m/s |
| 30-40 s | 0.0740 m | 0.4441 m/min | 0.0366 m/s |
| 40-50 s | 0.0723 m | 0.4339 m/min | 0.0357 m/s |
| 50-60 s | 0.0775 m | 0.4654 m/min | 0.0359 m/s |

分段结果显示漂移非常连续、近似线性，不像一次遥控器误触导致的突变。

### 10.3 复测结论

这次干净静止复测基本排除了 `/cmd_vel` 和 `go2w_bridge.py` 的影响：

- 没有运动命令链路。
- mode 一直为 `1`。
- gait_type 一直为 `0`。
- yaw_speed 一直为 `0`。
- position 仍以约 `0.447 m/min` 的速度漂移。
- velocity.xy 仍持续非零，均值约 `0.036 m/s`。

因此，`/lf/sportmodestate.position` 的静止漂移应视为该数据源当前状态下的固有问题或坐标/估计源问题，而不是上一轮遥控操作的偶然影响。

更新后的 EKF 建议：

1. 不融合 `/lf/sportmodestate.position`。
2. 不把 `/lf/sportmodestate.velocity` 作为高权重速度真值。
3. 若要使用 `/lf/sportmodestate.velocity.x`，只能低权重、短时辅助，并且必须和 RF2O 做同步对比。
4. 对 `/lf/sportmodestate` 更有价值的方向仍是 yaw/yaw_speed/IMU 姿态，而不是 position。
