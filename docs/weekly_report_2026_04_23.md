# Weekly Report: GO2W Real Navigation

**Date:** 2026-04-23  
**Project:** `go2w_real`  
**Scope:** 近 5 天 session 汇总（2026-04-19 ~ 2026-04-23）

---

## 本周摘要

本周的工作重点不是继续堆功能，而是把 GO2W 实机导航链路从“能跑、但现场现象复杂”推进到“可解释、可验证、可部署、可继续收敛”的状态。

这几天的核心成果主要有：

- 明确了 `motion_executor` 的推进方向，不再追求一次性做全，而是收敛为 `active-minimal` 路线。
- 建立并稳定了 headless 导航执行链路，包括 `start_navigation_headless.sh`、`navigation_executor.py`、POI/Waypoint、WebSocket 回传。
- 解决了几个现场最容易卡住的问题，包括：
  - 近目标点不完成
  - 到点后状态没有回到 `idle`
  - Nav2 配置和运行态参数不一致
  - 小角速度导致机器狗动作不自然
- 补了一批回归测试，把参数、状态机、桥接逻辑和 Waypoint 行为固定下来，降低后续调参和回归成本。

整体来看，本周最大的进展不是“多做了几个功能点”，而是把导航链路的权威入口、排障路径和验证方式逐步收敛清楚了。

---

## 本周重点解决的问题

| 问题 | 现场表现 | 解决方式 | 当前结果 |
|---|---|---|---|
| `motion_executor` 目标过大 | 一开始同时想处理 active、recovery、final yaw，集成风险过高 | 设计上收缩为 `active-minimal`，先只保证 rotate / drive / stop 和 `/cmd_vel` 接管 | 路线更清晰，验证成本显著下降 |
| 小角速度动作不自然 | 很小的 `angular.z` 也会让 GO2W 出现点步、跺步感 | 在 `go2w_bridge.py` 增加 `angular_deadband`，直接在桥接层做命令裁剪 | 获得了一个低风险、易调试的现场调参抓手 |
| Waypoint 接近目标点但不完成 | 人眼看已经到目标附近，但 CLI 导航仍然等待结束 | 在 `navigate_to_waypoint.py` 中增加 near-goal completion 逻辑和阈值参数 | Waypoint 导航体验更贴近现场预期 |
| Headless 启动依赖人工顺序 | 多终端手工启动，容易残留旧进程或顺序错误 | 建立 `start_navigation_headless.sh`，串起驱动、SLAM、Nav2、executor | 已形成可部署的 headless 启动雏形 |
| 到点后不回 `idle` | `remaining_distance` 已很小，但状态仍停在 `navigating` | 现场读取 `/controller_server` 运行态参数，发现 Foxy 实际使用的是旧 `goal_checker` 命名空间 | 已将 `goal_checker` 与 `general_goal_checker` 显式对齐 |
| 配置看似正确但行为不符 | YAML 中容差写得很宽，但实际运行仍卡在目标附近 | 不只看源码配置，同时读取 launch 生成的临时参数和 ROS 运行态参数 | 建立了更可靠的参数排障方法 |

---

## 本周是如何解决这些问题的

### 1. 先收缩问题，而不是继续扩展问题

本周一个很重要的判断是：`motion_executor` 这条线如果继续同时追求完整语义，就很容易陷入“每个问题都沾一点，但没有一个点真正闭环”的状态。

因此本周先把目标缩成 `active-minimal`：

- `shadow` 继续保留，负责观察和验证
- `active` 只先负责接管真实 `/cmd_vel`
- recovery 和严格 final yaw 明确延后

这一步的价值在于把“真正要解决的问题”从复杂系统问题，收敛回动作控制问题。

### 2. 先在桥接层解决动作体验问题

针对实机上“小角速度也会触发明显动作”的问题，本周没有先去深挖 DWB critic，而是选择在 `go2w_bridge.py` 增加 `angular_deadband`。

这个思路的优点是：

- 改动范围小
- 可快速现场试值
- 直接作用在最终送进 Unitree SDK 的命令上

这类问题很适合用桥接层控制点快速收敛，而不是上来就改动整条 Nav2 控制链。

### 3. 用行为层补齐“近目标点”语义

在 Waypoint 导航上，本周识别到一个很真实的现场问题：机器人已经接近目标点，但从操作体验上仍然“不算完成”。

这里采用的不是单纯调大 Nav2 容差，而是给 `navigate_to_waypoint.py` 增加近目标点判定参数，让终端单点导航更符合人对“到达”的理解。

这体现出一个思路：同样是“到点”，CLI 工具、WebSocket 执行器和 Nav2 action 的职责可以不同，但必须各自明确。

### 4. 把部署态入口真正串起来

本周 headless 链路的收敛很关键。以前这条链路更多是“多个命令可以拼起来”，现在已经逐步变成“一条受控启动路径”：

- `xt16_driver`
- `slam_rf2o.launch.py`
- `navigation_executor.py`
- WebSocket 指令/回传

同时，关于 headless 统一配置的设计也明确下来了：后续应当用单一 YAML 作为运行时权威来源，而不是让 shell 参数、环境变量、launch 参数、waypoint 文件各自分散生效。

### 5. 不再只信源码配置，而是把运行态参数纳入排障流程

本周一个非常关键的现场结论是：

- 源码里的 `nav2_params_foxy.yaml` 写的是 `general_goal_checker.xy_goal_tolerance = 0.50`
- 但运行中的 `/controller_server` 实际暴露出来的是 `goal_checker.xy_goal_tolerance = 0.25`

也就是说，问题不在于“参数没写”，而在于“Foxy 运行时真正吃的是另一套命名空间”。

这次排障的突破点就在于同时查看了：

- 源码配置
- launch 生成的临时 YAML
- 运行中的 ROS 参数
- `/navigate_to_pose/_action/feedback`

最终确认是运行态 `goal_checker` 没对齐，导致机器人看起来已经到点，但 Nav2 action 始终没有 `SUCCEEDED`，`navigation_executor` 也就不会回到 `idle`。

---

## 本周体现出的我的思路

从这几天的推进方式来看，这一周体现出的主要思路有：

- 优先解决真实机器人上最影响体验和稳定性的核心问题，而不是先追求功能完整。
- 明确区分开发态和部署态，避免一套脚本同时承担调试和上线两种职责。
- 对“统一配置源”和“参数一致性”高度敏感，已经不满足于“能调通”，而是在往“可复现、可维护、可交接”收敛。
- 遇到现场问题时，不是只改参数，而是希望把问题上升为设计问题和流程问题，比如 headless 统一配置、状态机职责分层、入口收敛。
- 对真实行为和运行态结果的重视程度很高，说明整个工作已经从“代码视角”转向“系统视角”。

---

## 我这边的观察和想法

结合本周的工作，我这边有几个判断：

- `motion_executor` 现在最值得继续坚持的是“小步接管”策略。`shadow first, active later` 已经证明是正确方向。
- Foxy 下很多 Nav2 问题都不适合只靠读 YAML 判断，运行态参数检查应该成为固定动作。
- `navigation_executor.py` 和 `start_navigation_headless.sh` 已经具备雏形，下一步最值得做的是把它们彻底统一到一份 headless runtime config 上。
- 现在系统不缺入口，真正缺的是“哪个入口才是权威来源”。这正是后续 headless unified config 的价值所在。
- 很多现场体验问题未必都要下沉到 Nav2 内核层，桥接层、小工具层和部署入口层其实都有很大优化空间。

---

## 值得继续发掘的工作流

### 1. Spec -> Plan -> 实现 -> 验证 -> 文档沉淀

本周已经形成了比较清晰的节奏：

- 先出设计文档
- 再写实施计划
- 再做最小实现
- 用 focused pytest 和现场验证确认行为
- 最后补说明文档和使用文档

这条工作流很适合继续保持，因为它能防止“现场灵感式改动”不断侵蚀系统边界。

### 2. Shadow -> Active-Minimal -> 完整语义

这是一条很适合机器人控制链路的工作流：

- `shadow` 先证明判断逻辑是对的
- `active-minimal` 先证明接管动作是对的
- 后面再逐步把 recovery、final yaw 等复杂语义补回来

它的优点是每一阶段都有明确验收目标，不会一下子把问题混在一起。

### 3. 终端工具先验证，再接 WebSocket 中枢

建议继续保留这样的分层：

- 终端 Waypoint / CLI 先验证局部行为
- mock sender / 单点发送器验证协议链路
- 最后再和中枢系统对接

这样可以避免“机器人端和中枢端一起改，最后谁的问题都说不清”的情况。

### 4. 配置、运行态、action 三层同时看

本周的调试已经证明了一个很实用的工作流：

- 先看源码配置
- 再看 launch 生成的临时参数
- 再读运行中 ROS 参数
- 最后看 action feedback / status

这套方法对 Nav2、Foxy、实机链路都非常适用，值得固化下来。

---

## 值得借鉴的经验

- 不要只相信源码里的 YAML，真实行为最终以运行态参数和 action 状态为准。
- 机器人“看起来到了”不等于导航“已经成功完成”，最终要看 `NavigateToPose` 是否 `SUCCEEDED`。
- 当问题规模过大时，先收缩出一个能闭环验证的最小版本，往往比一次做全更快。
- 很多实机体验问题不一定要通过大改 Nav2 才能解决，桥接层和工具层往往也能提供高价值控制点。
- Headless 一键启动的价值不只是省命令，更重要的是减少残留进程、错误顺序和人工记忆负担。
- 开发态和部署态最好明确分离，否则调试工具很容易污染上线入口。

---

## 本周关键记录

- `2026-04-20`  
  - `docs/superpowers/specs/2026-04-20-go2w-motion-executor-active-minimal-design.md`
  - 明确 `active-minimal` 路线

- `2026-04-21`  
  - `docs/superpowers/specs/2026-04-21-go2w-bridge-angular-deadband-design.md`
  - `a21b133 Improve GO2W Nav2 goal handling and executor control`

- `2026-04-22`  
  - `8ec41ad Fix near-goal waypoint completion`
  - `d6530f7 feat(go2w_real): add real nav bringup and stabilize waypoint navigation`

- `2026-04-23`  
  - `78f1f84 docs: add GO2W headless navigation design spec`
  - 现场排查并确认 Foxy 运行态 `goal_checker` 与源码配置不一致

---

## 下周建议

- 继续推进 headless unified config，让 headless 栈有单一配置源。
- 把 `go2w_auto_explore` 中同类 Nav2 goal checker 命名问题也一起排查统一。
- 固化一套“运行态参数检查”脚本或检查清单，避免同类问题重复靠人工排查。
- 继续用 `shadow -> active-minimal` 路线推进 `motion_executor`，不要过早恢复全部复杂语义。

