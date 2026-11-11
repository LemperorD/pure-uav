# 深度测距与任务接入（现行实现）

- 更新：2026-11-11
- 状态：已落地，随 `c0af177 接通相机深度与制导` 合入 `dev/main-loop`
- 相关：[depth-guidance.md](depth-guidance.md)（采集/标定/投影细节）、[camera-ipc.md](camera-ipc.md)（共享内存协议）

---

## 1 结论

目标距离**不在 main 里解算，也不由 VIO 暴露查询接口**。检测进程（`detect_rt`）用同一帧的
原生深度做完投影后，把 `has_range / range_m` 写进目标包；main 只负责**接收、门控、转发**。

早期方案（`MissionService` 调 `rangeAtPixel(u, v, t)`）已废弃，原因见 §8。

```text
camera_server
  OrbbecCamera::rangeCalibration()             // SDK 深度内参 + D2C 外参 + 彩色 Kalibr cam0
  → StreamMeta{has_depth, range_calibration_valid,
               depth K/D, color_fx/fy/cx/cy, color_distortion[8],
               R_c2d, t_mm, depth_width/height}
  → VIO 通道(gray + depth_mm) / 彩色通道(depth_mm | MJPG)

detect_rt（独立进程）
  pure_uav::camera::rangeCalibration(stream)   // C2D 求逆为 D2C，标定不全时返回空
  DetectionRuntime 工作线程：
    DepthRangeProjector::update(depth_mm, depth_us)    // 一帧深度只正向投影一次
    fillTargetDepthRange(result, projector, color_us)  // 每帧先清零，再按框中心取 9×9 中值
  → FrameResult.has_range / range_m
  → MissionTargetPacket.has_range / range_m   → socketpair

main / MissionService
  MissionTargetChannel::latest()               // 校验 finite、> 0、时间戳单调
  TargetAdapter::make()                        // 制导侧去畸变 + range 透传
  MissionController::targetUsable()            // has_range && range_m > 0 才允许确认/进 Strike
  GuidanceLoop                                 // has_range 进 GuidanceState（PPN 用 LOS 差分）
```

---

## 2 距离语义

| 项 | 约定 |
| --- | --- |
| 定义 | **彩色相机光心到目标表面点的欧氏距离**，不是深度相机 Z，也不是光轴深度 |
| 单位 | 米 |
| 采样点 | 检测框中心 `(u_px, v_px)`，原始彩色像素域 |
| 同帧性 | 距离与目标框来自**同一帧**；深度按设备时间差映射到该彩色帧的时钟域 |
| 邻域 | 彩色域 9×9 中值，至少 5 个有效点；前后景混叠（spread 超限）时拒绝输出 |
| 范围 | 软件过滤 0.15–10 m，超范围不 clamp，直接判无效 |
| 时间差 | 彩色/深度设备时间差 > 50 ms 判无效 |
| 失败 | 逐帧清零：`has_range=false, range_m=0`；目标框仍可继续发布 |
| 撞击确认 | **距离小不算撞击成功**，`strike_confirmed` 仍需外部信号（传感器/裁判/仿真事件） |

实现位置：[depth_range.cpp](../../../src/vio/estimator/src/depth_range.cpp)、
[target_depth_range.hpp](../../../src/vio/estimator/include/pure_uav/target_depth_range.hpp)。

---

## 3 main 侧接入点

| # | 位置 | 职责 |
| --- | --- | --- |
| 1 | [mission_target_channel.hpp](../../../src/common/mission_target_channel.hpp) | 收包；拒绝非有限、`range_m <= 0`、时间戳回退或过期 |
| 2 | [mission_service.cpp](../../../src/app/glue/mission_service.cpp) | 取最新包；`hostToFcuUs` 把采集时刻映射到飞控 boot 时钟；组 `GuidanceTargetSample` |
| 3 | [target_adapter.cpp](../../../src/app/glue/target_adapter.cpp) | 按 `guidance-camera.yaml` 对像素去畸变；**只透传**有效 range，不补算、不外推 |
| 4 | [mission_controller.cpp](../../../src/app/mission/mission_controller.cpp) | `targetUsable()` 要求 `has_range && range_m > 0`；Strike 中丢失则保持位置，`target_lost_timeout_s` 后回 Search |
| 5 | [guidance_loop.cpp](../../../src/guidance/flight/guidance_loop.cpp) | `has_range/range_m` 写入 `GuidanceState`，PPN 用 LOS 差分算 ω |

任务层行为（`MissionConfig` 默认值见 `config/mission.yaml`）：

| 场景 | 任务行为 |
| --- | --- |
| 搜索时有框但无距离 | 不计确认帧，不进入 Strike；`search_timeout_s` 后返航 |
| Strike 中距离丢失（目标框仍在） | 立即停止制导，定点保持；`target_lost_timeout_s`（默认 0.5 s）内恢复则继续 |
| Strike 中持续无距离 | 回 Search，重新确认 |
| 检测进程退出 | main 请求任务返航（保留 VIO） |
| 相机/VIO 断流 | 任务停止 + `NAV_LAND` 兜底（见 `landing_failsafe`） |

---

## 4 启动期诊断

- `rangeMeasurementError(stream)`（[guidance_camera.hpp](../../../src/app/runtime/guidance_camera.hpp)）
  返回空表示测距可用，否则返回中文原因：
  - `has_depth != 1`：`camera.yaml` 的 `use_depth=false` 或深度流未开启；
  - `range_calibration_valid != 1`：相机服务没有完整的深度内参/D2C 外参/彩色标定。
- `main --mission`（会向飞控发指令）在相机启动后立即检查：测距不可用则打印原因并**退出**，
  避免起飞后白搜 30 s 再返航。台架联调可加 `--no-send`，此时只告警、继续运行。
- `guidanceCameraMatchesStream()` 在制导未标定或 `range_calibration_valid != 1` 时提前返回成功：
  这两种情况 `StreamMeta.color_fx` 等字段不可信，无法核对彩色内参，而测距本来也不可用；
  制导未标定时 `guidance_ready=false`，任务不会进入 Strike。

---

## 5 板端验收

1. **停旧进程再重建**：V2 共享内存协议与旧版不兼容，`camera_server / main / vio_live / detect_rt`
   必须一起重启；`/dev/shm` 建议 ≥ 64 MiB。
2. `./bin/camera_server --check-config`，确认启动日志没有 `深度外参查不到`；
   启动后应有 `[Orbbec] 深度→彩色 平移` 输出。
3. `./bin/detect_rt --json` 观察 `has_range / range_m`：
   - 已知距离的平面/装甲板：1 m、2 m、3 m、5 m；
   - 画面中心与边缘各测一次（验证畸变/D2C）；
   - 遮挡、反光、目标边缘、相机重启后应变为 `has_range=false` 而不是给出旧值。
4. `./bin/main --mission --no-send --check-config` 先看配置；
   再 `--mission --no-send` 跑台架，确认启动期测距检查的日志符合预期。
5. 记录：range 误差（RMS / 最大）、有效率、深度与彩色时间差、Strike 是否进得去。
6. 注意 `config/guidance-camera.yaml` 仍为 `calibrated: false`：测距可以单独验收，
   但制导不会运行、任务不会撞板；相机安装标定完成后才能飞撞板流程。

无硬件测试覆盖：`depth_range`（投影/邻域/时间差）、`rgbd_pipeline`（真实 IPC + 目标包）、
`detection_runtime`（深度所有权与失效清零）、`target_bridge`、`guidance_camera`、
`mission`（距离门控与 Strike 中丢失）、`mission_wire`。

```bash
cmake -S . -B build && cmake --build build -j2
ctest --test-dir build --output-on-failure \
  -R '^(depth_range|rgbd_pipeline|detection_runtime|target_bridge|guidance_camera|mission|mission_wire|target_adapter)$'
```

---

## 6 待确认与风险

1. **真机 D2C 是否可用**：部分 Orbbec 型号/固件拿不到深度→彩色外参，此时测距恒不可用。
   启动日志出现 `深度外参查不到，目标测距不可用` 时要先解决标定，不能靠软件猜。
2. **两份彩色标定必须同源**：测距用 `config/camera.yaml` 的
   `camera.range_color_calibration_path`，制导用 `config/guidance-camera.yaml`。
   当前两者数值一致（fx≈1367.4658、4 个 radtan 系数、1920×1080）；
   连接相机后 main 会再核对一次，不一致则启动失败。
3. `StreamMeta.color_fx` 等字段与 `range_calibration_valid` 绑定，没有独立的“彩色标定有效位”。
   如果以后要单独校验证制导彩色内参，需要扩展 IPC 协议（当前不做）。
4. **TPN/APN 仍被启动参数限制**：`guidance_session.cpp` 只允许
   `PPN + FROM_LOS_DIFFERENCE + t_go_freeze=0`。现在有测距了，放宽是制导策略决定，
   需要单独评估，不在本阶段改动。

---

## 7 早期 `rangeAtPixel` 方案为何废弃

原方案是 main 调用 VIO 的 `rangeAtPixel(u_px, v_px, captured_ns)`，要求：

- main 在 50 Hz 主循环里持 `VioRuntime::PublicationGuard` 调用，接口必须另用独立锁；
- 深度帧匹配、彩色→深度投影、会话失效全部由 main 侧接口负责；
- 需要新增查询状态、统计和跨会话清理。

实际落地时发现：检测进程本来就同时持有同帧的彩色与深度，配对天然、无跨进程时序问题，
且 `DepthRangeProjector` 是纯 OpenCV 依赖，和 IMU/飞控解耦。于是改为**检测侧算完随包携带**：

- main 不再需要读深度、不再新增锁或线程；
- 失败语义简单：拿不到就 `has_range=false`，不会把旧距离配给新框；
- 代价是测距与制导各持一份彩色标定，必须靠 §4 的启动校验绑死同源。
