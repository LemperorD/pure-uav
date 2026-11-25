# 系统架构

> 本文说明模块分层、依赖方向、运行期数据流和线程模型。
> 具体坐标/时间契约见 [interfaces.md](interfaces.md)；app 层细节见 [app/README.md](app/README.md)。

---

## 1 分层

```text
main（全局入口）
 └─ app/guidance_application（GuidanceApplication：CLI + 参数 + 分发）
     ├─ app/guidance_sim_tools（CSV + 合成世界，地面用）
     └─ guidance/flight（GuidanceLoop：飞行外壳）
         └─ guidance/adapter（外部口径 → GuidanceState）
             ├─ guidance_libs（纯算法）
             └─ common（vec3 / mavlink_units）
```

依赖方向只允许**从上到下**：

```text
app → guidance/flight → guidance/adapter → guidance_libs → common
app/guidance_sim_tools → guidance/flight
test → 任意被测层
```

- `guidance_libs` 不依赖 app、driver、I/O；
- `adapter` 不依赖 driver；
- `flight` 不依赖 app/guidance_sim_tools；
- `driver` 不依赖 guidance（未来真机胶水放 `src/app/glue/`，由 main 链接）。

---

## 2 模块与 target

| 模块 | target | 内容 |
|---|---|---|
| `src/common` | `pure_vec3` / `mavlink_units` / `pure_math_tools` | 向量、单位换算、数学小工具 |
| `src/driver` | `communicate` / `OrbbecSDK` | MAVLink 串口、相机 SDK |
| `src/guidance/guidance_libs` | `guidance_core` | `guidance_step` / `pn_law` / `quat` |
| `src/guidance/guidance_libs` | `guidance_sim` | 传感器墙 / 目标模型（**不上机**） |
| `src/guidance/adapter` | `guidance_state_adapter` | 坐标/姿态/时间换算 + 体检 |
| `src/guidance/flight` | `guidance_shell` | `GuidanceLoop` + `GuidanceRingLog` |
| `src/guidance` | `guidance_flight` | INTERFACE 聚合：core + adapter + mavlink_units + shell |
| `src/app/sim/guidance_sim_tools.{hpp,cpp}` | `guidance_sim_tools` | CSV 录制/回放 + 合成世界 |
| `src/app/startup/guidance_application.{hpp,cpp}` | `guidance_application` | CLI + 参数 + 分发 |
| `src/app/startup/guidance_example.cpp` | `guidance_example` | 制导调用示例入口 |
| `src/app/main.cpp` | `main` | 单串口、单相机的并发机载入口 |

`guidance_flight` 是机载最小足迹；`guidance_sim` 不在其中。

---

## 3 运行期数据流

### 3.1 启动

```text
main()                         # 并发机载运行
  → Communicator 接收线程（单串口）
  → OrbbecCamera 采集线程（单相机）
      → 灰度图队列 → VIO 线程 → Communicator::pushVisionPose
      → BGR 最新帧 → Detector 推理线程
  → 制导线程：飞控遥测 + 检测结果 → GuidanceLoop → Communicator::pushGuidanceCmd
  → 监督、停止、join
guidance_example              # 制导调用示例（独立可执行文件）
  → GuidanceApplication::run(argc, argv)
      → 默认值 + YAML + CLI → GuidanceAppParams
      → guidance_validate_params()
      → 选模式：
          合成世界 / 回放 → guidance_run_synthetic / guidance_run_replay
          真机           → 由 main.cpp 单独组合，不经此仿真入口
```

### 3.2 一拍（`GuidanceLoop::run_once`）

```text
① poll 飞控帧 / 目标帧（没有新帧 → 沿用上一帧）
② 目标像素 → 相机系 → 机体 → 世界（得到 los_hat）
③ adapter::GuidanceRawFrame → to_guidance_state()
④ quick_check()（不健康 → 按 on_unhealthy 降级）
⑤ guidance_step()（纯函数）
⑥ cmd_accel_to_ned()
⑦ sink->send()（每拍都发，PX4 offboard 要求持续到达）
⑧ 统计 / 延迟打点 / GuidanceRingLog 记录
```

关键点：

- `state.t` 取**目标帧采样时刻**，只在目标帧更新时重算（见 [decisions/0003](decisions/0003-los-rate-source.md)）；
- 没有新目标帧 → 重发上一拍；
- 不健康 → 保持/零速度悬停/停，但**不断流**（断流会让 PX4 掉出 offboard）。

### 3.3 端口

- `GuidanceInsSource`：飞控/仿真给 `GuidanceInsSample`；
- `GuidanceTargetSource`：检测/仿真给 `GuidanceTargetSample`（像素）；
- `GuidanceCommandSink`：MAVLink/ROS2/仿真接 `GuidanceOutput`。

主循环是**拉模型**：每拍主动 `poll`，数据到达时刻不打乱节拍。

---

## 4 线程与实时性

- 控制回路单线程：`GuidanceLoop::run()` 用绝对时间 `sleep_until`，不累积漂移；
- 循环内**零分配、零 printf、零 std::string、零文件 I/O**；
- `GuidanceRingLog` 只做内存 memcpy，dump 在循环外；
- CSV 录制/回放是地面工具，不参与机载实时路径；
- 将来的 `flight_logger` 应该是“循环内 push + 独立写线程”，不在控制线程里做 I/O。

---

## 5 仿真与真机

| 场景 | 入口 | 数据源/出口 |
|---|---|---|
| 合成世界 | `guidance_run_synthetic` | `GuidanceSyntheticWorld`（理想点质量） |
| CSV 回放 | `guidance_run_replay` | `GuidanceCsvReplaySource` |
| XTDrone2/Gazebo | `sim_xtdrone2/phase_b_guidance.py` | Python 节点 + `pure_guidance` shim |
| PX4 真机 | 未来 `src/app/glue/` | `MavlinkInsSource` + `MavlinkAccelSink` |

约束：

- `sim_xtdrone2` 只通过 shim 调 `guidance_libs`，不链接 app；
- 真机胶水才 include `communicator.hpp`，guidance 核心不依赖 driver；
- 机载二进制不链接 `guidance_sim` / `guidance_sim_tools`。

---

## 6 设计规则

1. **算法纯函数**：`guidance_step` 不读全局、不碰 I/O。
2. **边界只转一次**：坐标/姿态/时间只在 adapter 转。
3. **模块不反向依赖**：core 不依赖 app；app 不依赖 driver 的内部实现。
4. **机上不带仿真/工具**：`guidance_flight` 不含 sim，`main` 不链 tools（未来拆 `main_sim`）。
5. **文档跟读者走**：算法在 guidance，外壳/工具在 app，契约在 interfaces。

---

## 7 未来

- `flight_logger`：中间量/检测变量按间隔记录；
- 真机胶水：`MavlinkInsSource` / `MavlinkAccelSink`；
- VIO 融合：PX4 EKF2 EV → `vehicle_local_position`，guidance 不直接消费 VIO；
- 已拆出 `guidance_example`；未来真机入口 `main_real` 不链工具。
