# WebSocket 导航发送端说明

## 1. 适用场景

这份文档面向“发送端 / 中枢 / 导航管理模块”开发者。

当前系统里：

- 机器人侧运行 `go2w_real/scripts/navigation_executor.py`
- 发送端不在机器人本机
- 发送端需要在自己的机器上启动 WebSocket 服务端
- 机器人会主动连到发送端

当前机器人所在局域网 IP：

- `172.16.22.51`

这表示：

- 发送端可以在同一局域网内看到这台机器人
- 但发送端不要去直接连接 `172.16.22.51:8100`
- 正确模式是：发送端开服务，机器人主动连过来

## 2. 通信方向

当前导航接口的正确方向是：

```text
发送端（WebSocket 服务端）
    <- 机器人主动连接 -
机器人（navigation_executor.py）
```

不是下面这种：

```text
发送端 -> 直接连接机器人 172.16.22.51:8100
```

原因是：

- `navigation_executor.py` 是 WebSocket 客户端
- 它不会在机器人本机监听端口
- 它只会主动连接到你提供的 WebSocket 服务端地址

## 3. 发送端必须提供的服务

发送端需要提供一个 WebSocket 服务端，监听：

- 端口：`8100`
- 路径：`/ws/navigation/executor`

也就是完整地址形如：

- `ws://<sender_ip>:8100/ws/navigation/executor`

例如发送端机器 IP 是 `172.16.22.100`，则：

- 发送端服务地址：`ws://172.16.22.100:8100/ws/navigation/executor`

机器人侧启动时应连接这个地址：

```bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

## 4. 联调前准备

### 4.1 发送端侧准备

发送端机器需要：

- 与机器人处于同一局域网
- 防火墙允许 `8100` 端口访问
- 启动自己的 WebSocket 服务

### 4.2 机器人侧准备

机器人侧至少要完成：

1. 启动雷达
2. 启动 `slam_rf2o.launch.py`
3. 启动 `navigation_executor.py`

机器人侧参考命令：

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
source install/setup.bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

## 5. 最小调用流程

建议按下面顺序联调：

1. 发送端先启动 WebSocket 服务端
2. 机器人运行 `navigation_executor.py` 并连接到发送端地址
3. 发送端确认收到来自 `172.16.22.51` 的连接
4. 发送端下发 `navigate_to`
5. 接收 `on_progress`
6. 到达后接收 `on_arrived`
7. 如果需要中止，则发送 `abort_navigation`

## 6. 发送端最小可运行示例

下面是一个最小 Python 示例，使用 `websockets` 包实现发送端服务。

仓库里已经提供了一个现成测试脚本：

- `go2w_real/scripts/navigation_sender_test_poi001.py`

它会：

- 在发送端机器上启动 WebSocket 服务端
- 等待机器人 `navigation_executor.py` 连接
- 连接建立后自动下发 `POI_001`
- 持续打印 `on_progress / on_arrived / on_error`
- 在测试结束后退出并返回退出码

直接运行示例：

```bash
python3 navigation_sender_test_poi001.py --announce-ip 172.16.22.100
```

如果发送端服务 IP 正好是 `172.16.22.100`，机器人侧对应命令就是：

```bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

安装依赖：

```bash
python3 -m pip install websockets
```

示例代码：

```python
#!/usr/bin/env python3
import asyncio
import json
import uuid

import websockets


HOST = "0.0.0.0"
PORT = 8100
PATH = "/ws/navigation/executor"

connected_clients = set()


async def send_json(ws, payload):
    await ws.send(json.dumps(payload, ensure_ascii=False))


async def send_navigate_to(ws, target_id, request_id=None, sub_id=1):
    if request_id is None:
        request_id = f"req_{uuid.uuid4().hex[:8]}"

    payload = {
        "action": "navigate_to",
        "request_id": request_id,
        "sub_id": sub_id,
        "target_id": target_id,
    }
    await send_json(ws, payload)
    print("sent:", payload, flush=True)
    return request_id


async def send_abort(ws, request_id):
    payload = {
        "action": "abort_navigation",
        "request_id": request_id,
    }
    await send_json(ws, payload)
    print("sent:", payload, flush=True)


async def receiver_loop(ws):
    async for raw in ws:
        data = json.loads(raw)

        if data.get("type") == "ping":
            await send_json(ws, {"type": "pong"})
            print("recv ping -> sent pong", flush=True)
            continue

        if data.get("type") == "pong":
            print("recv pong", flush=True)
            continue

        print("recv event:", data, flush=True)


async def interactive_loop(ws):
    current_request_id = None
    loop = asyncio.get_running_loop()

    while True:
        text = await loop.run_in_executor(
            None,
            input,
            "\n输入命令(nav POI_001 / abort / quit): ",
        )
        text = text.strip()

        if not text:
            continue

        if text in ("quit", "exit", "q"):
            await ws.close()
            return

        if text.startswith("nav "):
            target_id = text[4:].strip()
            current_request_id = await send_navigate_to(ws, target_id)
            continue

        if text == "abort":
            if not current_request_id:
                print("当前没有可中止的 request_id", flush=True)
                continue
            await send_abort(ws, current_request_id)
            continue

        print("未知命令", flush=True)


async def handler(ws, path):
    if path != PATH:
        print(f"unexpected path: {path}", flush=True)
        await ws.close(code=1008, reason="invalid path")
        return

    peer = ws.remote_address
    print(f"robot connected from: {peer}", flush=True)
    connected_clients.add(ws)

    try:
        recv_task = asyncio.create_task(receiver_loop(ws))
        input_task = asyncio.create_task(interactive_loop(ws))
        done, pending = await asyncio.wait(
            [recv_task, input_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()
    finally:
        connected_clients.discard(ws)
        print("robot disconnected", flush=True)


async def main():
    print(f"starting sender server at ws://{HOST}:{PORT}{PATH}", flush=True)
    async with websockets.serve(handler, HOST, PORT, ping_interval=None):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
```

## 7. 如何启动发送端

假设上面的代码保存为 `navigation_sender_demo.py`，启动方法：

```bash
python3 navigation_sender_demo.py
```

启动后会监听：

- `ws://0.0.0.0:8100/ws/navigation/executor`

如果发送端机器实际 IP 是 `172.16.22.100`，那么机器人就应连接：

- `ws://172.16.22.100:8100/ws/navigation/executor`

## 8. 如何判断机器人已经连上

当机器人侧执行：

```bash
ros2 run go2w_real navigation_executor.py \
  --server-uri ws://172.16.22.100:8100/ws/navigation/executor
```

发送端会看到类似日志：

```text
robot connected from: ('172.16.22.51', 54321)
```

这里的 `172.16.22.51` 就是机器人当前局域网地址。

## 9. 发送端如何下发导航

交互示例里，连接建立后可以直接在发送端终端输入：

```text
nav POI_001
```

它会发送：

```json
{
  "action": "navigate_to",
  "request_id": "req_xxxxxxxx",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

当前机器人默认 waypoint 文件在：

- `/home/unitree/ros_ws/src/go2w_real/config/go2w_waypoints.yaml`

当前默认 POI 示例：

- `POI_001`
- `POI_002`

## 10. 发送端如何中止导航

在交互示例里输入：

```text
abort
```

它会发送：

```json
{
  "action": "abort_navigation",
  "request_id": "req_xxxxxxxx"
}
```

成功中止后，发送端通常会收到：

```json
{
  "event_type": "on_progress",
  "request_id": "req_xxxxxxxx",
  "sub_id": 1,
  "remaining_distance": 0.0,
  "status": "paused"
}
```

## 11. 发送端会收到哪些消息

### 11.1 运行中进度

```json
{
  "event_type": "on_progress",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "remaining_distance": 1.5,
  "status": "running"
}
```

### 11.2 到达目标

```json
{
  "event_type": "on_arrived",
  "request_id": "req_20260402_001",
  "sub_id": 1
}
```

### 11.3 错误

```json
{
  "event_type": "on_error",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "error_message": "Nav2 action server is not available within 10.0s"
}
```

### 11.4 心跳

机器人侧可能会发：

```json
{
  "type": "ping"
}
```

发送端应回：

```json
{
  "type": "pong"
}
```

## 12. 发送端接入业务系统时的建议

实际接入时，建议发送端内部做这些事情：

- 为每个导航任务生成唯一 `request_id`
- 保存 `request_id -> 业务任务` 的映射
- 在收到 `on_arrived` 后更新任务成功状态
- 在收到 `on_error` 后触发重试或人工介入
- 在收到 `on_progress` 后更新前端或日志
- 处理机器人断线重连

## 13. 常见问题

### 13.1 发送端服务已启动，但机器人没有连上

检查：

- 发送端 IP 是否可被机器人访问
- `8100` 端口是否被防火墙拦截
- 机器人侧 `--server-uri` 是否写对
- WebSocket 路径是否是 `/ws/navigation/executor`

### 13.2 机器人连上了，但一发导航就报错

如果收到：

```json
{
  "event_type": "on_error",
  "error_message": "Nav2 action server is not available ..."
}
```

说明问题不在发送端，而在机器人本地导航栈。

需要先确认机器人侧已经启动：

```bash
ros2 launch go2w_real slam_rf2o.launch.py
```

### 13.3 收不到 `on_arrived`

优先检查：

- 目标 `target_id` 是否存在
- waypoint 坐标是否正确
- 机器人侧地图、TF、costmap、Nav2 是否正常

### 13.4 是否可以由发送端主动发起连接到机器人

当前不可以。

当前协议落地方式固定为：

- 发送端是 WebSocket 服务端
- 机器人是 WebSocket 客户端

## 14. 推荐联调顺序

1. 发送端先启动 WebSocket 服务端
2. 机器人启动 `slam_rf2o.launch.py`
3. 机器人启动 `navigation_executor.py --server-uri ws://<sender_ip>:8100/ws/navigation/executor`
4. 发送端确认收到来自 `172.16.22.51` 的连接
5. 发送端先发 `nav POI_001`
6. 观察是否收到 `on_progress`
7. 到达后确认 `on_arrived`
8. 再测试 `abort_navigation`
