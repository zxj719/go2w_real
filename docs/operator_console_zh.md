# 端侧 Operator Console 指南（手机选 POI → 直接导航）

> 本文写给以后在 GO2W 机器人端侧工作的 agent。目标：让你不用再重读源码就能独立完成
> “部署、启动、排障、二次开发”这套闭环。

---

## 1. 它是什么

`operator_console/` 是一个在机器人上跑的小 HTTP 服务，加一个静态网页：

- 手机 / PC 在同一局域网打开 `http://<robot-ip>:8080` 看到一个控制台
- 控制台里可以：
  1. 启动 / 停止 **GO2W 导航栈**（xt16_driver + slam_rf2o + Nav2，localization 模式）
  2. 从 YAML 里的 POI 列表中**点一下，机器人就去**（一次一个目标）
  3. 看 Foxglove Web（`ws://<robot-ip>:8765`）做可视化
- **不**连任何外网 / 云端 / 导航调度服务器；一切都发生在机器人局域网内

它从 `raw/g1_nav/operator_console/` 移植而来，但在 POI 分发这块做了扩展：g1 那版只负责
启动导航栈，目标由外部 WebSocket server 下发；GO2W 这版把 POI 派发做进 HTTP 里，
靠本地 `navigate_to_waypoint.py` 发 Nav2 goal。

> **同时兼容旧路径**：出于调度对接需要，白名单里也保留了启动老版
> `start_navigation_headless.sh` 的 profile——里面会附带启动 `navigation_executor.py`
> 连远端 WebSocket 调度服务。两种模式并存：手机端一键切，但**不能同时跑**（见 §9）。

---

## 2. 目录结构（要知道每个文件干嘛）

```
operator_console/
├── launch_manager.py        # HTTP 服务主进程（systemd 跑这个）
├── profiles.yaml            # 白名单：允许哪些命令能被 launch_manager 起
├── web/
│   ├── index.html           # 手机控制台（含 POI 选择、状态、日志）
│   └── foxglove-layout.json # 推荐的 Foxglove 布局（map/costmap/scan/plan）
├── systemd/
│   ├── launch-manager.service
│   └── foxglove-bridge.service
└── deploy.sh                # 从开发机 rsync + 安装 systemd + 重启服务

scripts/
└── start_navigation_no_server.sh   # 新的无服务器版导航栈起点脚本
```

关键约束：

- `launch_manager` **不允许**执行 `profiles.yaml` 里没写的命令——要加新能力必须往
  `profiles.yaml` 里加条目，不能走 HTTP 传入任意 shell。
- POI 派发走 `profiles.yaml` 顶层的 `navigation.navigate_command` 模板，字面量
  `{{waypoint}}` 会被替换成用户点的那个 POI id。其他字段（waypoint-file、action-name
  等）都来自环境变量或模板内固定写死。

---

## 2.1 Profiles 速查（手机下拉框里能看到的东西）

| Profile 名 | 背后的 shell 命令 | 跑起来有什么 | 手机 POI 能用吗 |
|-----------|------------------|--------------|----------------|
| `navigation` | `scripts/start_navigation_no_server.sh` | xt16 + slam_rf2o(localization) + Nav2（`zt_0` 地图） | **可以**，本地派发 |
| `navigation_test_map` | `... no_server.sh test_1` | 同上但加载 `test_1` 地图 | 可以（记得把 `waypoint_file` 切到 `go2w_waypoints.yaml`）|
| `navigation_headless_ws` | `scripts/start_navigation_headless.sh` | 上面那套 + `navigation_executor.py` 连远端 `ws://192.168.123.186:8100/ws/navigation/executor` | **不行**：executor 占住 Nav2 action；POI 派发会被 reject / preempt |
| `navigation_headless_ws_test_map` | `... headless.sh test_1` | 同 ws 版，只是 `test_1` 地图 | 同上，不要用 POI 选择 |

选型建议：

- 你就是用手机选 POI 走 → `navigation`（或 `navigation_test_map`）
- 你要对接**外部调度服务**（公司里那套 WebSocket 导航中枢） → `navigation_headless_ws`
- 不要同时 Start 两个 profile；launch_manager 允许同时运行，但 Nav2 action 只有一份，
  谁先连 server 谁就赢，另一路会 reject。系统性地避免方法是：**切换前先 Stop 上一条**。

---

## 3. 运行时数据流

启动导航栈：

```
手机 POST /api/profiles/navigation/start
    ↓
launch_manager 用 bash -lc 拉起
  scripts/start_navigation_no_server.sh
    → xt16_driver
    → ros2 launch go2w_real slam_rf2o.launch.py (localization, no RViz)
    → 等 bt_navigator 起来后挂住
```

点 POI：

```
手机 POST /api/nav/goto {"target_id": "POI_012"}
    ↓
NavigationDispatcher.start("POI_012")
  1. 如果已有子进程在跑 → SIGTERM 它的进程组 → 4s 后还在就 SIGKILL
  2. 以 GO2W_WAYPOINT_FILE=<profiles.yaml 里配的> 起新子进程：
       bash -lc "source foxy + unitree + install && \
                 exec ros2 run go2w_real navigate_to_waypoint.py \
                      --waypoint-file $GO2W_WAYPOINT_FILE \
                      --waypoint POI_012"
  3. 子进程 stdin = /dev/null，所以发完一次 goal 就会 EOFError 退出
  4. 后台线程读 stdout，从日志里抓 dist= 和 reached/aborted 字样，更新状态
    ↓
手机轮询 GET /api/nav/status 得到 {state, target_id, remaining_distance, error}
```

停止或切换：

```
POST /api/nav/cancel         → 只杀 navigate_to_waypoint 子进程
POST /api/profiles/.../stop  → 先 cancel 上面那个，再杀整条栈
systemd stop launch-manager  → SIGTERM 处理器里先 navigator.shutdown() 再 stop_all()
```

**为什么这么设计**：

- 单实例，好预测：任何时候最多一个 POI 子进程，新目标自动抢占旧目标。
- 不复用一个常驻 `navigate_to_waypoint.py`，因为那脚本是交互式的（`input()` 循环），
  多段复用需要喂 stdin + 改脚本；一次一个进程更干净，出错也不会影响下一次。
- 进程组 + `start_new_session=True` 确保 SIGTERM 会扩散到 bash + ros2 + python 子孙，
  不会留僵尸。Linux 上走 `killpg`，Windows（仅开发机单测）走 `terminate()`。

---

## 4. HTTP API 速查

所有路径相对 `http://<robot-ip>:8080`，无鉴权（只暴露到局域网）。

| Method | Path | 说明 |
|--------|------|------|
| GET  | `/api/health` | 所有 profile + `_navigation` 的当前状态 |
| GET  | `/api/profiles` | profile 列表（含描述、当前状态）|
| GET  | `/api/profiles/{name}/status` | 某条栈的进程状态 |
| GET  | `/api/profiles/{name}/logs?tail=N` | 栈子进程 stdout 最近 N 行（默认 100）|
| POST | `/api/profiles/{name}/start` | 启动栈 |
| POST | `/api/profiles/{name}/stop` | 停止栈（会先取消 POI 任务）|
| GET  | `/api/waypoints` | 从 YAML 加载的 POI 列表 |
| GET  | `/api/nav/status` | 当前 POI 任务状态：`idle/starting/running/succeeded/failed/canceled` |
| GET  | `/api/nav/logs?tail=N` | navigate_to_waypoint 最近 N 行输出 |
| POST | `/api/nav/goto` `{"target_id": "POI_012"}` | 派发 POI 目标 |
| POST | `/api/nav/cancel` | 取消当前 POI 目标 |

`/api/nav/status` 返回示例：

```json
{
  "ok": true,
  "state": "running",
  "target_id": "POI_012",
  "started_at": "2026-04-24T07:31:02.123+00:00",
  "uptime_seconds": 4.1,
  "remaining_distance": 1.23,
  "pid": 12045
}
```

终态（`succeeded/failed/canceled`）会多出 `exit_code`、`finished_at`，失败时还会有
`error` 字段（从 stdout 里抓到的错误行）。

---

## 5. 部署

### 5.1 首次部署

开发机上：

```bash
cd <...>/go2w_real/operator_console
# 修改 deploy.sh 里的默认 IP（GO2W_IP 或第一个参数），默认 192.168.123.18
./deploy.sh              # 用默认 IP
./deploy.sh 192.168.1.42 # 指定 IP
./deploy.sh --check      # 不改机器人，只打印要做的事
```

`deploy.sh` 会：

1. rsync `operator_console/` 到 `~/ros_ws/src/go2w_real/operator_console/`
2. rsync `scripts/start_navigation_no_server.sh` 到 `~/ros_ws/src/go2w_real/scripts/`
3. 安装 `ros-foxy-foxglove-bridge`、`python3-yaml`（如缺）
4. 拷贝 systemd unit → daemon-reload → enable
5. 重启两个 service，打印 status

### 5.2 重复部署 / 回滚

- 改了代码就再跑一次 `./deploy.sh`——rsync 带 `--delete`，旧文件会被清掉。
- 回滚：`git checkout <老版本> && ./deploy.sh`。

### 5.3 手动部署（无网盘/无 ssh key 的应急路径）

机器人上：

```bash
sudo cp ~/ros_ws/src/go2w_real/operator_console/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now foxglove-bridge launch-manager
```

---

## 6. 配置：要改哪里

### 6.1 换地图 / 换 POI 文件

编辑机器人上的 `~/ros_ws/src/go2w_real/operator_console/profiles.yaml`：

```yaml
navigation:
  waypoint_file: /home/unitree/ros_ws/src/go2w_real/config/go2w_map_waypoints.yaml
```

想换成 `test_1` 地图：

1. 把 `waypoint_file` 改成 `go2w_waypoints.yaml`
2. 在 UI 里切换到 `navigation_test_map` profile 再 Start

改完 `profiles.yaml` 要 `sudo systemctl restart launch-manager` 重读。

### 6.2 换端口

改 `systemd/launch-manager.service` 里 `--port 8080`，然后 `daemon-reload + restart`。
手机端网址里的端口也记得同步改。

### 6.3 换外部 WebSocket 调度服务 URI（仅对 `navigation_headless_ws*` 生效）

`start_navigation_headless.sh` 默认连 `ws://192.168.123.186:8100/ws/navigation/executor`，
换服务器有两种路径：

- **改一次就生效**：编辑机器人上的 `profiles.yaml`，给那两个 profile 的 `env` 里加
  `GO2W_SERVER_URI: "ws://x.x.x.x:8100/ws/navigation/executor"`，然后
  `sudo systemctl restart launch-manager`。
- **只改 command 行**：在 profile 的 `command` 后面追加 `--server-uri ws://...`
  即可，这种改动更显式但每条 profile 都要改一遍。

其他可调的都在 `start_navigation_headless.sh --help` 列出来了（heartbeat、reconnect
delay、near-goal distance 等）。不要走 HTTP 动态传参——白名单是整条命令固定文本。

### 6.4 换 ROS_DOMAIN_ID

两处要一起改才能互通：

- `systemd/foxglove-bridge.service` 的 `Environment=ROS_DOMAIN_ID=0`
- `profiles.yaml` 每个 profile 的 `env.ROS_DOMAIN_ID`

### 6.5 把 navigate_to_waypoint 的 near-goal 阈值 / 重试次数调开

改 `profiles.yaml` 顶层的 `navigate_command`，往 `ros2 run ...` 后面追参数即可：

```yaml
navigation:
  navigate_command: >
    source ... &&
    exec ros2 run go2w_real navigate_to_waypoint.py
    --waypoint-file "$GO2W_WAYPOINT_FILE"
    --waypoint "{{waypoint}}"
    --near-goal-distance 0.35
    --abort-replan-retries 2
```

注意 **`{{waypoint}}` 这个字面量必须保留**，否则 launch_manager 启动时会报
`navigate_command template is missing the '{{waypoint}}' placeholder`。

---

## 7. 排障 SOP

### 7.1 手机打不开页面

1. 确认手机和机器人同一网段（ping robot ip）
2. `systemctl status launch-manager` 有没有在跑？
3. `curl http://localhost:8080/api/health`（在机器人上）能不能通？
4. `sudo journalctl -u launch-manager -n 200` 看报错

### 7.2 Start 栈之后 POI 还是派发不动

1. `GET /api/nav/logs?tail=200`——通常会看到：
   - `Nav2 action server is not available within 10.0s` → 栈还没起全，等
     `bt_navigator` 起来，或者栈根本没起来
   - `failed to load waypoint file` → waypoint_file 路径不对
   - `unknown waypoint '...'` → POI id 在 YAML 里没有
2. 机器人上 `ros2 node list | grep bt_navigator` 验证 Nav2 是否真的 ready
3. `ros2 action list | grep navigate_to_pose` 看 action 是否在
4. 如果 `state` 一直 `running` 但 `remaining_distance` 没动 → planner 可能卡了，
   走 `/api/nav/cancel` 然后检查 TF（`/tf`, `/tf_static` 是否有
   map→odom→base_link 链）

### 7.3 navigate_to_waypoint 报 "another instance is already running"

那个脚本（以及 navigation_executor.py）自己会在 `/tmp/go2w_navigation_executor.lock`
上抢 flock。但 `navigate_to_waypoint.py` 本身并不用这个 lock，会用 lock 的是原先的
`navigation_executor.py`。如果同时启动了旧 websocket 版执行器，就会占住 Nav2
action server——表现是你的 POI goal 被立即 reject 或 preempt。

处理：

```bash
# 看谁占着 action
ros2 action info /navigate_to_pose -t
# 找出旧 executor
pgrep -fa navigation_executor.py
# 杀掉
pkill -f navigation_executor.py
rm -f /tmp/go2w_navigation_executor.lock
```

### 7.4 stop 完发现还有残留进程

说明 trap 没跑全（最常见是 SIGKILL 被 systemd 直接下发导致 bash 的 trap 没机会）。
`launch-manager.service` 里已经写 `KillMode=mixed` + `TimeoutStopSec=20`，正常给足
了时间。如果还是有残留：

```bash
pgrep -fa start_navigation_no_server
pgrep -fa xt16_driver
pgrep -fa slam_rf2o
# 确实有残留就手动 kill，然后回来修代码：可能是某个步骤新起了子进程没 trace 到
```

### 7.5 手机上 Foxglove 连不上

1. `systemctl status foxglove-bridge`
2. 确认端口 8765 没被占（`ss -tlnp | grep 8765`）
3. Foxglove Web 有时 https 页面连 ws:// 会被 mixed-content 拦；
   用手机浏览器打开 `http://app.foxglove.dev` 而不是 https 版本，或用 Foxglove 桌面/移动 App

---

## 8. 二次开发：常见改动怎么下手

### 8.1 想加一个「回原点」按钮

- `profiles.yaml` 里 POI 文件本来就有 `POI_012 / wp_00 @ (0,0)`，已经够用。
- 只在 `web/index.html` 里加个按钮，`onclick` 调 `/api/nav/goto` 传 `POI_012`。
- 不要往 `launch_manager.py` 里加新 endpoint——这种就是前端糖。

### 8.2 想让某个 POI 到达后自动执行一个动作（比如语音播报）

两种路径：

- **端侧在 arrive 后再派发一次**：在网页 JS 里监听 `state === 'succeeded'`，再
  `fetch('/api/nav/goto', ...)` 或者你自己的 API。写起来快，但逻辑放前端不太稳。
- **扩 launch_manager**：加一个 `/api/nav/playbook` endpoint，吃一组 POI 列表和
  每段到达后的命令，串行执行。需要改 `NavigationDispatcher` 成“任务链”抽象。
  改动面较大，先和调用方对齐再动。

### 8.3 想加一个新的导航栈配置（比如开 RViz 的调试模式）

只需要在 `profiles.yaml` 的 `profiles:` 下加一个新 entry：

```yaml
profiles:
  navigation_debug:
    command: /home/unitree/ros_ws/src/go2w_real/scripts/start_navigation_headless.sh
    description: Navigation stack with WebSocket executor (legacy debug path)
    env:
      ROS_DOMAIN_ID: "0"
```

launch_manager 重启后会自动出现在下拉框里。**不要**给 command 传用户输入的参数——
白名单整条命令只允许固定文本。

### 8.4 想做鉴权

当前是零鉴权（LAN only）。加鉴权最小改动：

- `LaunchManagerHandler` 里加一个装饰器，检查 `Authorization: Bearer ...` header；
  token 从环境变量 / 一个本地文件读。
- `deploy.sh` 里把 token 写进 `systemd` 的 `Environment=`。
- 前端登录页把 token 存 localStorage，每个 fetch 都带。

别上来就 TLS + JWT——机器人端侧 TLS 证书管理是另一个工程。

---

## 9. 和现有系统的关系

| 文件 | 关系 |
|------|------|
| `scripts/start_navigation_headless.sh` | 旧版：会 **另外**起 navigation_executor.py 连 WebSocket 调度服务。**现在也作为 `navigation_headless_ws` profile 暴露给 UI**（见 §2.1）。 |
| `scripts/start_navigation_no_server.sh` | 新版：**只**起 xt16 + slam_rf2o；POI 派发交给 operator_console。 |
| `scripts/navigation_executor.py` | 旧版执行器：WebSocket 客户端，由 `start_navigation_headless.sh` 拉起；**不要**和本地 POI 派发同时跑（会抢 Nav2 action，并且抢 `/tmp/go2w_navigation_executor.lock`）。 |
| `scripts/navigate_to_waypoint.py` | operator_console 给每个 POI 选择都起一个实例，`--waypoint XXX` 一次性发 goal。 |
| `config/go2w_map_waypoints.yaml` / `go2w_waypoints.yaml` | POI 源头。operator_console 里不做任何修改，只读。 |

---

## 10. 一次完整的实机演练 checklist

1. `ssh unitree@<robot-ip>`；`cd ~/ros_ws`
2. `colcon build --packages-select go2w_real` 后 `source install/setup.bash`
   （只在改了 package 或第一次时）
3. 开发机：`cd operator_console && ./deploy.sh <robot-ip>`
4. 机器人：`systemctl status launch-manager foxglove-bridge`，两个都 active
5. 机器人：`curl -s http://localhost:8080/api/health | jq` 看两个 service + _navigation
6. 手机连同一 WiFi，打开 `http://<robot-ip>:8080`
7. 下拉选 `navigation` → Start，等 Stack Status 从 `starting` → `running`
8. Stack Logs 里有 `[start_nav_no_server] stack is ready` 才算真的起来
9. POI 列表里挑一个不是 `POI_012`（原点）的目标，点一下
10. Nav 状态行从 `Running · POI_XXX` 到 `Succeeded · POI_XXX`；如果一直 running
    不动，看 Navigation Logs
11. 换一个 POI 点一下，观察有没有正确抢占（旧 PID 消失，新 PID 出现）
12. 点 Cancel，状态变 `Canceled`
13. Stop 栈，观察 Stack Status 回 `stopped`，Services 全部变灰

能顺利走完这 13 步，这套端侧 console 就可以交给别人用了。

---

## 11. 留给下一位 agent 的坑

- `_scrape_line_locked` 里抓 `"reached successfully"` / `"finished with status"` 的
  字面量是从 `navigate_to_waypoint.py` 的日志抠出来的。如果哪天把 navigate 脚本
  的日志文案改了，这边的成功/失败判定就会漏掉。改文案时一起改这里。
- `NavigationDispatcher` 目前假设任何新 POI 都无条件抢占旧 POI。如果以后要加「同一
  POI 重复点击忽略」或者「排队」，是加在 `start()` 最上面的判断里；别把逻辑塞到
  HTTP handler 里。
- systemd 重启 `launch-manager` 会导致 POI 子进程被一并收走（KillMode=mixed）。如果
  以后想让子进程独立于 manager 存活（比如 manager 热更新），要把子进程脱离 systemd
  scope，且要在启动时把它们的 pid 写到一个文件里好下次连上。现在**不要**这么干，
  意外残留成本比热更新收益高。
- UI 现在同时列出 `navigation*`（本地 POI 派发）和 `navigation_headless_ws*`（远端
  WebSocket 调度）两类 profile。两类**不能同时 Start**：Nav2 action 和
  `/tmp/go2w_navigation_executor.lock` 都是单占资源。如果想在 UI 里加互斥，可以在
  `ProcessManager.get_or_create(...).start()` 之前扫一遍所有 profile，见到 `running`
  的就拒绝。实现不难，但要先和现场使用流程对齐（有些场景会故意先 Stop 再 Start）。
