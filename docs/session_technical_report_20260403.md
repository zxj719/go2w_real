# GO2W 实机导航链路技术报告

## 1. 摘要

本次 session 的目标不是单点修 bug，而是把 GO2W 实机上的一整条导航链路从“能跑一部分、现象复杂、问题互相干扰”推进到“可解释、可复现、可切换地图、可由外部系统调用”的状态。

最终完成的核心结果包括：

- 将实机导航链路从仿真/历史依赖中剥离，收敛到 `go2w_real`
- 建立稳定的实机导航主链路：
  - `xt16_driver`
  - `/unitree/slam_lidar/points`
  - `pointcloud_to_laserscan`
  - `laser_filters`
  - `rf2o_laser_odometry`
  - `slam_toolbox`
  - `Nav2`
- 实现两套可切换地图组合：
  - 默认：`go2w_map_waypoints.yaml + zt_0.data + zt_0.posegraph`
  - 可选：`go2w_waypoints.yaml + test_1.data + test_1.posegraph`
- 建立 waypoint 记录、终端导航、WebSocket 导航执行、发送端测试、CPU profiling、无 RViz 一键启动等完整工具链
- 形成了较明确的排障方法论：先识别混杂因素，再分层隔离变量，再通过组合验证收敛到稳定策略

本报告重点记录：

- 如何实现
- 尝试过哪些路径
- 哪些路径失败以及失败原因
- 如何识别混杂因素、隔离变量
- 如何选择策略、做组合验证
- 最终攻克了哪些功能和技术点

## 2. 目标与约束

### 2.1 目标

本次工作的实际目标逐步演化为以下几个层次：

1. 让 `go2w_real` 能在 ROS 2 Foxy 下独立启动
2. 建立一条对实机可用的 SLAM + Nav2 导航链路
3. 解决 RViz 显示、TF、地图、odom、目标点执行之间的联动问题
4. 形成 waypoint 采集和调用工具
5. 提供一个可被外部“发送端/中枢”通过 WebSocket 调用的导航执行接口
6. 再向前推进到无 RViz 的一键启动形态，便于实际部署

### 2.2 约束

- 平台：Ubuntu + ROS 2 Foxy
- 机器人：Unitree GO2W
- 传感器主输入：`/unitree/slam_lidar/points`
- 用户明确要求：
  - 不做自动站立
  - 遥控优先，不在导航链路里捆绑 teleop
  - 最终需要由外部系统通过 WebSocket 调用
- 已知环境问题：
  - 官方某些 odom / slam 话题存在但不稳定，甚至不发布
  - Foxy 与较新 ROS 2 launch / Nav2 接口存在兼容差异
  - RViz 闪退、TF 丢帧、目标执行失败等问题会彼此叠加，容易误判

## 3. 初始问题与系统症状

在 session 初期，系统呈现出多层问题同时出现的状态：

- `go2w_real/launch/nav2.launch.py` 无法启动
  - `ParameterFile` 导入失败
  - 仍引用 `go2w_office_sim`
- RViz 中看不到机器人或地图
- `slam_toolbox` 报 `frame 'lidar' ... reason 'Unknown'`
- 发布导航点后 RViz 闪退，但导航进程本身未中断
- `/unitree/slam_lidar/points` 可用，但 odom 来源不确定
- `navigate_to_waypoint.py` 在 Foxy 上因为反馈字段缺失崩溃
- WebSocket 调用需求尚未落地

这些问题的共同特点是：现象重叠，彼此干扰，不能靠“继续调一个参数”来解决，必须先拆层。

## 4. 总体实现策略

本次工作的核心策略不是“从上往下硬推”，而是采用四层拆解：

1. **先清依赖和入口**
   - 确保 `go2w_real` 在 Foxy 下能独立启动
   - 去掉实机链路中不该依赖的仿真包和旧路径
2. **再固定主链路**
   - 先选一条最可信的 odom / SLAM / Nav2 组合
   - 不在多个 odom 来源之间摇摆
3. **再补工具链**
   - waypoint 记录
   - 终端导航
   - CPU profiling
   - WebSocket 执行端与发送端
4. **最后做部署化**
   - 无 RViz
   - 一键启动
   - 地图组合切换

这套顺序的重点是：每次只增加一层复杂度，避免“SLAM、Nav2、RViz、WebSocket 一起上”的耦合爆炸。

## 5. 实施过程与技术路线

### 5.1 实机包与旧依赖剥离

最早的问题是 `go2w_real` 仍带着仿真依赖思维：

- launch 中引用了 `go2w_office_sim`
- 使用了 Foxy 中并不存在的 `ParameterFile`

处理方式：

- 直接把 `go2w_real` 与 `go2w_auto_explore`、仿真路径语义切开
- 让 `go2w_real` 只保留实机需要的 launch / config / scripts
- README 和文档路径一并整理成拆分后的结构

这一步解决的是“入口不纯”的问题。  
如果入口本身仍依赖仿真包，那么后续所有导航问题都没有排障意义。

### 5.2 odom 策略选择：从多来源摇摆，收敛到单一路线

#### 5.2.1 尝试过的候选来源

本次 session 中被明确排查或讨论过的 odom 相关来源包括：

- 官方 slam / odometer 相关话题
- `rt/odommodestate` / `rt/lf/odommodestate`
- `/sportmodestate`
- hybrid velocity
- lidar 自带相关话题
- RF2O 基于 `/scan` 的激光里程计

#### 5.2.2 失败与放弃的原因

失败并不是“接口不存在”，而是“不可作为稳定工程依赖”：

- 某些官方话题虽然名字存在，但没有样本或不连续
- 某些状态量在机器人静止/运动时表现不稳定
- hybrid velocity 明确被观察到没有值
- 某些官方 SLAM 相关 topic 在当前环境中并没有按照预期出现

#### 5.2.3 最终策略

最终收敛到：

- 点云输入使用 `/unitree/slam_lidar/points`
- 通过 `pointcloud_to_laserscan + laser_filters` 得到 `/scan`
- 用 `rf2o_laser_odometry` 产生 `/odom` 和 `odom -> base`

选择理由：

- 数据链路可观测
- 话题存在且可测频率
- 不依赖官方 odometer 服务当前是否稳定
- 更适合作为一条“可重复部署”的工程主链路

### 5.3 TF / RViz / 地图显示问题的排障方法

这一阶段最大的难点不是单个 bug，而是多个显示层和计算层混在一起。

#### 5.3.1 典型症状

- RViz 无机器人、无地图
- `Message Filter dropping message: frame 'lidar' ... reason 'Unknown'`
- 发布导航点后 RViz 闪退
- global costmap 显示异常

#### 5.3.2 关键识别方法

这里用了一个非常重要的变量隔离方法：

- **先区分“RViz 挂了”和“导航栈挂了”**

用户反馈中有一个关键现象：

- RViz 闪退
- 但导航程序没有中断

这说明：

- 问题不一定在 Nav2 主流程
- 有可能是 RViz Display 配置、TF 显示链、特定显示项触发的问题

于是后续策略不是一味重调 Nav2，而是：

- 调整 RViz 中显示项组合
- 去掉易触发问题的 Odometry 显示
- 增加更直接有价值的 Global Costmap / Path 显示

#### 5.3.3 TF 门控

为了避免“链路还没就绪就启动下游”，后续在 launch 中引入了等待 TF 的门控逻辑：

- `wait_for_transform.py`
- 先等 `odom <- lidar`
- 再等 `map <- lidar`
- 通过后再放行 `slam_toolbox`、Nav2、RViz

这一步非常关键，因为它把“启动时序问题”从随机现象变成了可解释的 gating 条件。

### 5.4 SLAM + Nav2 主链路收敛

最终形成的主链路为：

```text
/unitree/slam_lidar/points
-> pointcloud_relay.py
-> /cloud_relayed
-> pointcloud_to_laserscan
-> /scan_raw
-> laser_filters
-> /scan
-> rf2o_laser_odometry
-> /odom
-> slam_toolbox
-> /map
-> Nav2
```

核心落点是 [slam_rf2o.launch.py](/home/unitree/ros_ws/src/go2w_real/launch/slam_rf2o.launch.py)。

这条链路的价值在于：

- 数据输入单一
- odom 生成路径清晰
- TF 链可验证
- Nav2 启动条件可控

### 5.5 Nav2 参数调优

本次调参不是从空白开始，而是参考了同环境下可跑通的人形参数思路，但保留 GO2W 自身的几何和运动约束。

#### 5.5.1 保留的 GO2W 特性

- footprint 不改成人形或圆形近似
- local / global costmap 保留贴合实机的 footprint 语义
- 速度、旋转速度和膨胀半径围绕 GO2W 实际室内表现调整

#### 5.5.2 观察到的问题

- 狭窄区域无法掉头
- 旋转过快导致叠图
- global costmap 显示异常或不合理
- Nav2 `send_goal failed`

#### 5.5.3 调整方向

- 减小 costmap 以提高狭窄空间内的可通行性
- 下调最大旋转速度，减少 scan matching 叠图
- 稳定 costmap / path / local planner 的组合显示与行为
- 通过 TF gating 和 scan 链路稳定性修正 `send_goal` 前提

#### 5.5.4 关键认知

这里最大的经验是：

- 很多“导航不动”并不是 controller 参数本身错误
- 而是上游 TF、scan、odom、costmap 的某一层不稳定

也就是说，Nav2 参数调优必须建立在“感知和定位链路已稳定”的前提上，否则只是在噪声上调参数。

### 5.6 waypoint 工具链实现

这部分最终形成了两个工具：

1. waypoint 记录工具
2. waypoint 终端调用工具

#### 5.6.1 waypoint 记录工具

最初需求是“保存地图并记录点”，后续收敛为：

- 不再由该脚本保存地图
- 只负责记录固定导航点
- 记录流程明确为：
  1. 先记录 RViz `Publish Point` 的位置
  2. 再记录机器人当前朝向
  3. 最后输入地点名称

同时输出路径也被固定到更明确的位置。

#### 5.6.2 terminal waypoint 导航工具

`navigate_to_waypoint.py` 最初存在一个 Foxy 兼容性问题：

- 代码尝试读取 `NavigateToPose_Feedback.estimated_time_remaining`
- 但 Foxy 该字段不存在
- 导致运行时直接崩溃

处理方式：

- 对反馈字段做版本兼容判断
- 保留 `distance_remaining` / `navigation_time`
- 有字段就显示，没有就跳过

后续又继续增强：

- 支持按 `id / name / index` 选点
- 对齐默认 waypoint 文件路径

### 5.7 CPU profiling 工具

为了后续在 `htop` 之外做可解释的 CPU 归因，补充了 `profile_nav_stack.py`。

这个工具的价值不只是“看 CPU”，而是把运行中的多个进程按组件聚合：

- `go2w_bridge`
- `rf2o_laser_odometry`
- `slam_toolbox`
- `controller_server`
- `planner_server`
- `bt_navigator`
- `rviz2`
- `navigate_to_waypoint`
- 后续又加入 `navigation_executor`

这一步的工程价值在于：

- 可以用统一口径观察系统负载
- 方便对比“有 RViz / 无 RViz”“有 WebSocket / 无 WebSocket”等组合
- 为后续性能瓶颈判断提供基础

### 5.8 WebSocket 导航执行接口实现

这是本次 session 的一个关键里程碑。

#### 5.8.1 目标

实现一个“导航执行端”，满足：

- 主动连接到 `ws://[server]:8100/ws/navigation/executor`
- 接收 `navigate_to`
- 接收 `abort_navigation`
- 上报 `on_progress`
- 上报 `on_arrived`
- 上报 `on_error`
- 支持随时中断和重启

#### 5.8.2 实现策略

没有采用 shell 封装 `ros2 action`，而是直接在 Python 中做了：

- `websockets` 客户端
- Nav2 `NavigateToPose` action client
- 显式状态机
- 心跳与自动重连
- 任务抢占与取消

这样做的原因是：

- shell 方案难以可靠处理中断和状态同步
- 直接用 action client 更容易做反馈和取消闭环
- 后续更容易被中枢系统集成

#### 5.8.3 状态机

状态机最终收敛为：

- `DISCONNECTED`
- `CONNECTING`
- `IDLE`
- `STARTING`
- `NAVIGATING`
- `ABORTING`
- `ERROR`

#### 5.8.4 遇到的技术问题

1. 默认缺少 `websockets` 运行时依赖
2. 一开始的实现中事件队列和 asyncio loop 绑定错误
3. 退出时 `ActionClient` 析构会产生 `InvalidHandle`
4. 如果消息处理和导航启动串在一起，容易让 WebSocket 接收阻塞

#### 5.8.5 解决方式

- 补 `python3-websockets` 依赖
- 将 bridge event queue 放到运行中的 loop 内初始化
- 在 shutdown 中显式销毁 action client / node / executor
- 使用后台 task 处理消息和导航启动，避免把接收循环卡死
- 用 command version + 当前 task token 控制抢占和过期任务

#### 5.8.6 验证方式

这个功能没有只停留在代码审查层，而是做了实际 mock 验证：

- 本地起一个 mock WebSocket 服务端
- 让机器人执行端连接进来
- 发送 `navigate_to`
- 在 Nav2 不可用时验证 `on_error`
- 再验证 `abort_navigation -> paused`

这一步证明的是：

- WebSocket 协议方向正确
- action client 行为正确
- 中断状态能闭环

### 5.9 发送端说明与测试脚本

在接口落地后，又补了发送端视角的文档和脚本。

关键澄清点是：

- `navigation_executor.py` 是客户端
- 发送端必须是 WebSocket 服务端
- 不是发送端去直接连 `172.16.22.51:8100`

在此基础上补充了：

- 执行端中文文档
- 发送端中文文档
- `navigation_sender_test_poi001.py`

其中发送端测试脚本的作用是：

- 在另一台机器上起 WebSocket 服务
- 等待机器人连接
- 自动发送 `POI_001`
- 观察 `on_progress / on_arrived / on_error`

### 5.10 多地图组合与无 RViz 一键启动

最后阶段的目标是把系统从“开发态工具链”推进到“部署态入口”。

实现方式：

- 为 `slam_rf2o.launch.py` 增加 `slam_map_file` 可覆盖参数
- 不再通过手改 YAML 切地图
- 定义两套组合：
  - 默认：
    - `config/go2w_map_waypoints.yaml`
    - `map/zt_0.data`
    - `map/zt_0.posegraph`
  - 可选：
    - `config/go2w_waypoints.yaml`
    - `map/test_1.data`
    - `map/test_1.posegraph`
- 新增 `start_navigation_headless.sh`

这个脚本会自动：

1. 起 `xt16_driver`
2. 起 `slam_rf2o.launch.py use_rviz:=false`
3. 等待 `bt_navigator` 就绪
4. 再起 `navigation_executor.py`

这样做的关键是：

- 不是简单并行起三个进程
- 而是引入“Nav2 ready”门槛，减少时序混乱

## 6. 失败尝试与失败原因

本次 session 中有价值的失败主要包括：

### 6.1 直接复用旧 launch / 仿真依赖

失败原因：

- Foxy 兼容性不满足
- 实机 launch 仍引用仿真包
- 入口本身不纯

### 6.2 依赖官方 odom / slam 话题作为主方案

失败原因：

- 话题存在但不一定发布
- 某些状态量不可作为稳定工程输入
- 难以保证部署一致性

### 6.3 在上游链路不稳时直接调 Nav2 参数

失败原因：

- 问题源头不在 controller / planner
- TF、scan、odom、costmap 不稳时，调参数没有意义

### 6.4 把 RViz 问题误判为 Nav2 主流程问题

失败原因：

- RViz 闪退与导航进程退出不是同一件事
- 如果不先分离显示层与执行层，就会误调

### 6.5 WebSocket 方向理解错误

潜在失败点：

- 容易误以为发送端应直接连接机器人 IP

实际结论：

- 执行器是客户端
- 发送端必须起服务端

### 6.6 `set -u` 直接 source Foxy setup

失败现象：

- `AMENT_TRACE_SETUP_FILES: unbound variable`

失败原因：

- Foxy setup 脚本会直接访问未定义变量

解决方案：

- 在一键脚本中使用 `safe_source`
- 临时关闭 `nounset` 再 source

## 7. 混杂因素识别与变量隔离方法

这是本次 session 中最重要的方法论产出之一。

### 7.1 先分层，不先下结论

整个系统被拆成：

- 传感器层
- scan / odom 层
- TF 层
- SLAM 层
- Nav2 层
- RViz 显示层
- WebSocket 调度层

任何问题都先问：

- 它属于哪一层？
- 这一层的输入是否可信？
- 它是否只是另一层问题的表现？

### 7.2 通过“是否继续运行”区分问题域

例如：

- RViz 闪退但导航进程还在

就可以推断：

- 执行层未必出问题
- 显示层优先排查

### 7.3 通过“话题存在”与“话题可用”分离事实

一个核心经验是：

- 话题 list 里有，不等于工程可用
- 必须继续验证：
  - 是否有数据
  - 是否连续
  - 是否在运动时响应
  - 是否与目标功能匹配

### 7.4 通过 mock 把外部系统变量剥离

WebSocket 接口联调中，使用本地 mock 服务端替代真实中枢：

- 这样可以先验证执行器协议逻辑
- 排除真实发送端业务逻辑的干扰

### 7.5 通过“最小可运行组合”避免同时调多件事

例如：

- 先用 terminal waypoint 脚本直接打 Nav2
- 再接 WebSocket

这样可以先验证：

- waypoint 是否正确
- Nav2 是否能走

而不是把 waypoint、Nav2、WebSocket 一起混在一次测试里。

## 8. 组合验证策略

本次 session 中采用了多层组合验证。

### 8.1 静态验证

- `python3 -m py_compile`
- `bash -n`
- `colcon build --packages-select go2w_real`

### 8.2 运行时链路验证

- `ros2 topic list`
- `ros2 topic hz`
- `tf2_echo`
- `ros2 node list`

### 8.3 功能级验证

- RViz `2D Goal Pose`
- `navigate_to_waypoint.py`
- `save_map_and_waypoints.py`
- `navigation_executor.py`
- `navigation_sender_test_poi001.py`

### 8.4 协议级验证

- mock WebSocket 服务端
- `navigate_to -> on_error`
- `abort_navigation -> paused`

### 8.5 部署级验证

- 无 RViz headless 启动脚本
- 双地图组合切换
- Nav2 ready 后再启动执行端

## 9. 已实现功能清单

截至本次 session，已经完成或落地的能力包括：

### 9.1 导航主链路

- 实机 `slam_rf2o.launch.py`
- 无 RViz 模式运行
- 可切换 serialized slam 地图前缀

### 9.2 waypoint 能力

- waypoint 记录
- waypoint 终端列出
- waypoint 终端导航
- 支持 `id / name / index`

### 9.3 SLAM / Nav2 工具

- TF 等待门控
- CPU profiling
- 参数与 footprint 收敛

### 9.4 WebSocket 能力

- 导航执行器
- 心跳
- 自动重连
- 抢占
- 中断
- 到达 / 错误 / 进度回传

### 9.5 发送端能力

- 中文接口文档
- 中文发送端文档
- 单点自动测试脚本 `POI_001`

### 9.6 部署能力

- 无 RViz 一键启动
- 默认 / 可选地图组合

## 10. 关键技术点与攻克点

本次真正被攻克的技术点主要有：

1. **Foxy 兼容性修正**
   - launch / feedback 字段 / 生命周期时序

2. **odom 选型收敛**
   - 从多候选、不稳定的官方链路，收敛到 RF2O 主链路

3. **TF 启动门控**
   - 从随机时序问题变成明确条件控制

4. **RViz 与执行层问题解耦**
   - 防止显示问题误导导航调参

5. **WebSocket + Nav2 action 深度集成**
   - 不是 demo，而是带状态机、取消、重连的可用实现

6. **部署形态收敛**
   - 从多个手工命令收敛到 headless 一键启动

## 11. 当前推荐工作流

当前推荐工作流为：

1. 启动 `xt16_driver`
2. 启动 `slam_rf2o.launch.py`
3. 确认 TF / scan / odom / map 正常
4. 使用 terminal waypoint 脚本或 RViz 先验证 Nav2
5. 再启动 `navigation_executor.py`
6. 外部发送端通过 WebSocket 下发 `POI_xxx`

部署场景下则直接使用：

- `start_navigation_headless.sh`

默认地图组合：

- `go2w_map_waypoints.yaml + zt_0`

可选地图组合：

- `go2w_waypoints.yaml + test_1`

## 12. 当前风险与后续建议

虽然系统已经具备较高完成度，但仍有几个建议保留：

### 12.1 建议继续保留 RF2O 作为主线路

除非官方 odom 话题经过持续验证证明稳定，否则不建议切回多来源混用。

### 12.2 建议保留 mock sender 测试

每次改 WebSocket 协议或执行器逻辑时，都先用 mock sender 验证，不要直接把中枢和机器人一起改。

### 12.3 建议继续分离“开发态”和“部署态”

- 开发态：
  - RViz
  - profiling
  - terminal waypoint
- 部署态：
  - headless
  - WebSocket executor

### 12.4 建议补充更系统的回归测试矩阵

后续可进一步形成如下组合表：

- `zt_0 / test_1`
- `RViz / no RViz`
- `terminal waypoint / websocket`
- `normal nav / abort nav`

这样会把系统从“能用”推进到“可持续维护”。

## 13. 结论

本次 session 的最大成果，不只是把某个 launch 拉起来，而是完成了以下工程转变：

- 从依赖混杂、症状叠加、难以定位
- 转变为链路明确、工具完备、接口可调用、部署可复用

更重要的是，本次工作形成了一套稳定的方法：

- 先识别层次
- 再隔离变量
- 再选择单一路线
- 再做组合验证
- 最后做部署化封装

这使得 GO2W 实机导航链路不再只是“碰巧跑通”，而是进入了“可解释、可演化、可集成”的状态。
