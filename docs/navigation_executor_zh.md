# WebSocket 导航执行器说明

## 1. 角色说明

当前 `go2w_real/scripts/navigation_executor.py` 的角色是“导航执行端”。

它的工作方式是：

- 机器人这一侧主动发起 WebSocket 连接
- 连接到“导航管理模块 / 中枢”提供的 WebSocket 服务端
- 接收 `navigate_to` 和 `abort_navigation` 指令
- 调用本机 Nav2 的 `/navigate_to_pose`
- 回传 `on_progress`、`on_arrived`、`on_error`

这意味着：

- 当前实现是客户端，不是服务端
- 发送端不能直接“连到本机 `172.16.22.51` 上调用 `navigation_executor.py`”
- 正确方式是：
  - 发送端先提供 WebSocket 服务
  - 机器人侧运行 `navigation_executor.py`
  - 机器人主动连接发送端

## 2. 当前局域网信息

当前机器人工作机 IP：

- `172.16.22.51`

这个 IP 的用途主要是：

- 让同一局域网内的开发机能够看到这台机器
- 方便你通过 `ping`、`ssh`、日志查看等方式确认连通性

但对当前 `navigation_executor.py` 来说，更关键的是“发送端服务所在 IP”。

例如：

- 如果发送端服务部署在 `172.16.22.100`
- 那机器人侧就应该连接：
  - `ws://172.16.22.100:8100/ws/navigation/executor`

## 3. 前置条件

在运行 WebSocket 导航执行器之前，需要先把机器人本地导航栈启动起来。

### 3.1 终端 1：启动雷达

```bash
cd /unitree/module/unitree_slam/bin
./xt16_driver
```

### 3.2 终端 2：启动 ROS 2 环境与导航栈

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source install/setup.bash
ros2 launch go2w_real slam_rf2o.launch.py
```

说明：

- `network_interface` 默认就是 `eth0`
- 正常情况下不需要显式添加 `network_interface:=eth0`
- 只有当网卡不是 `eth0` 时，才需要手动覆盖

### 3.3 可选：查看当前可导航 POI

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py --list-pois
```

当前默认读取的 waypoint 文件是：

- `/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml`

当前这份文件中可用的 POI 可以通过上面的命令查看。

## 4. 机器人侧如何启动 WebSocket 导航执行器

### 4.1 使用当前测试环境默认地址

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py
```

当前默认连接地址：

- `ws://192.168.0.131:8100/ws/navigation/executor`

### 4.2 如果发送端服务运行在同一局域网另一台机器

假设发送端服务 IP 为 `172.16.22.100`，则运行：

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

### 4.3 运行成功后的表现

终端通常会看到类似日志：

```text
[navigation_executor] state: disconnected -> connecting
[navigation_executor] state: connecting -> idle
[navigation_executor] connected to ws://172.16.22.100:8100/ws/navigation/executor
```

如果发送端下发导航指令，还会看到：

```text
[navigation_executor] state: idle -> starting
[navigation_executor] state: starting -> navigating
```

## 5. 发送端如何调用

发送端在自己的 WebSocket 服务端上，向已经连接进来的机器人会话发送 JSON 指令。

### 5.1 导航到某个 POI

发送：

```json
{
  "action": "navigate_to",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

字段说明：

- `action`: 固定为 `navigate_to`
- `request_id`: 本次任务的唯一请求 ID
- `sub_id`: 子任务编号
- `target_id`: 目标点 ID，对应 waypoint YAML 中的 `id`

当前默认 waypoint 示例：

- `POI_001`
- `POI_002`

如果 waypoint 中没有 `id` 字段，执行器会自动回退使用 `name`。

### 5.2 中断当前导航

发送：

```json
{
  "action": "abort_navigation",
  "request_id": "req_20260402_001"
}
```

执行器会：

- 尝试取消当前 Nav2 目标
- 向发送端回传一条 `on_progress`
- 其中 `status` 为 `paused`

### 5.3 心跳

机器人侧会主动发送：

```json
{
  "type": "ping"
}
```

发送端应返回：

```json
{
  "type": "pong"
}
```

如果发送端主动发 `ping`，机器人侧也会回复 `pong`。

## 6. 发送端会收到什么回调

### 6.1 导航进行中

```json
{
  "event_type": "on_progress",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "remaining_distance": 1.234,
  "status": "running"
}
```

### 6.2 到达目标

```json
{
  "event_type": "on_arrived",
  "request_id": "req_20260402_001",
  "sub_id": 1
}
```

### 6.3 发生错误

```json
{
  "event_type": "on_error",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "error_message": "Nav2 action server is not available within 10.0s"
}
```

### 6.4 中止后暂停

```json
{
  "event_type": "on_progress",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "remaining_distance": 0.0,
  "status": "paused"
}
```

## 7. 状态机行为说明

当前执行器支持以下状态：

- `disconnected`
- `connecting`
- `idle`
- `starting`
- `navigating`
- `aborting`
- `error`

行为规则：

- 收到新的 `navigate_to` 时，会抢占当前任务
- 当前任务会先尝试取消，再启动新任务
- 收到 `abort_navigation` 时，会中断当前任务
- 中断完成后，执行器回到 `idle`
- 断线后会自动重连

## 8. 常见调用方式

### 8.1 只查看机器人当前支持哪些 POI

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py --list-pois
```

注意：

- 这个命令只打印 POI 列表
- 不会建立持续 WebSocket 连接
- 不会启动导航执行

### 8.2 正式接入中枢

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

### 8.3 手动验证某个 POI 是否能被 Nav2 执行

```bash
cd ~/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2w_real navigate_to_waypoint.py --waypoint POI_001
```

这个命令不走 WebSocket，而是直接本地给 Nav2 发目标，适合排查：

- waypoint 坐标本身是否正确
- Nav2 是否能正常执行
- 当前地图 / TF / costmap 是否正常

## 9. 常见误区

### 9.1 误区：发送端直接连接 `172.16.22.51:8100`

当前不是这种模式。

`navigation_executor.py` 不是 WebSocket 服务端，它不会在本机 `172.16.22.51:8100` 上监听等待连接。

正确模式是：

- 发送端开服务
- 机器人主动连接发送端

### 9.2 误区：`--list-pois` 就等于启动执行器

不是。

`--list-pois` 只会：

- 读取 waypoint 文件
- 打印 POI 列表
- 立即退出

### 9.3 误区：只启动 `navigation_executor.py` 就可以导航

也不是。

必须先保证本机 Nav2 已经起来，也就是至少先启动：

```bash
ros2 launch go2w_real slam_rf2o.launch.py
```

否则执行器虽然能连上 WebSocket，但会回：

- `on_error`
- 典型错误为 `Nav2 action server is not available ...`

## 10. 推荐联调顺序

建议按下面顺序联调：

1. 启动 `xt16_driver`
2. 启动 `slam_rf2o.launch.py`
3. 用 `ros2 run go2w_real navigate_to_waypoint.py --waypoint POI_001` 本地验证 Nav2
4. 确认发送端 WebSocket 服务可用
5. 启动 `navigation_executor.py --server-uri ws://<sender_ip>:8100/ws/navigation/executor`
6. 由发送端下发 `navigate_to`
7. 观察是否收到 `on_progress / on_arrived / on_error`
