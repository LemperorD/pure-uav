# 变更记录（迁移 / 合并 / 文档整理）

> 代码注释不写日期和迁移历史，统一记在这里。

## 2026-11-01

### app 代码合并（commit `b2d3493`）

按“少文件、职责清楚”合并：

| 合并后 | 合并自 |
|---|---|
| `assets/docs/app/README.md` 对应的 `app_types.hpp` | `sample.hpp` + `output.hpp` + `sensor_source.hpp` + `command_sink.hpp` + `CameraModel/LoopConfig` |
| `guidance_loop.{hpp,cpp}` | `guidance_loop` + `ring_log` |
| `tools.{hpp,cpp}` | `csv_io` + `synthetic_world` + `run_synthetic/run_replay` |
| `application.{hpp,cpp}` | `params` + CLI + 模式分发 |
| `main.cpp` | 只留入口（5 行） |

### 文档迁移（commit `99f3333`）

| 原位置 | 新位置 |
|---|---|
| `src/app/README.md` | `assets/docs/app/README.md` |
| `src/guidance/README.md` | `assets/docs/guidance/README.md` |
| `src/guidance/后续注意问题.md` | `assets/docs/guidance/后续注意问题.md` |
| `assets/docs/guidance.md` | `assets/docs/guidance/任务书.md` |

### 代码位置迁移（commit `b86e135`）

| 原位置 | 新位置 |
|---|---|
| `src/app/app_types.hpp` | `src/guidance/flight/flight_types.hpp` |
| `src/app/guidance_loop.{hpp,cpp}` | `src/guidance/flight/` |
| `src/app/application.{hpp,cpp}` | `src/app/startup/` |
| `src/app/tools.{hpp,cpp}` | `src/app/tools/` |

新增 `guidance_shell` target；`guidance_flight` = `guidance_core + state_adapter + mavlink_units + guidance_shell`。

### 文档索引（commit `1525293`）

- 新增 `assets/docs/README.md`：按任务/代码模块索引；
- 根 `README.md` 增加文档入口。

### 注释拆分（本次）

- `quat.hpp` 的长历史 → [decisions/0001-quat-semantics.md](decisions/0001-quat-semantics.md)；
- `guidance_types.hpp` 的改名故事 → [decisions/0002-pure-prefix.md](decisions/0002-pure-prefix.md)；
- `guidance_loop` 的时间戳/ω → [decisions/0003-los-rate-source.md](decisions/0003-los-rate-source.md)；
- CSV 格式 → [app/csv-format.md](app/csv-format.md)。

### 接口/架构/测试文档（本次）

- `state_adapter` / `state_check` 的坐标/时间/体检长注释 → [interfaces.md](interfaces.md)；
- `guidance_loop` 的完整一拍流程 → [architecture.md](architecture.md)；
- 三个测试文件的覆盖矩阵 → [testing.md](testing.md)。

### 文档补齐（本次）

- 新增 [comment-style.md](comment-style.md)、[app/usage.md](app/usage.md)、[app/blackbox.md](app/blackbox.md)；
- 新增 [decisions/0004-freeze-criterion.md](decisions/0004-freeze-criterion.md)、[0005-time-base.md](decisions/0005-time-base.md)、[0006-camera-vio.md](decisions/0006-camera-vio.md)、[0007-target-models.md](decisions/0007-target-models.md)、[0008-sensor-wall.md](decisions/0008-sensor-wall.md)；
- `guidance_types` / `target` / `sensor` / `application` / `tools` / `guidance_loop` 文件头补文档链接。

### 收尾（本次）

- 清理 `src/app`、`src/guidance`、`src/test` 中的日期与 `★/⚠/✅/🔴`；
- 公开 API 补 Doxygen：`IInsSource` / `ITargetSource` / `ICommandSink` / `GuidanceLoop` / `Application` / `guidance_step`。

### 命名空间归位（本次）

- guidance 飞行外壳：`pure_uav::app` → `pure_uav::guidance::flight`；
- 输入适配层：`pure_uav::adapter` → `pure_uav::guidance::adapter`；
- `AppParams` 从 `flight_types.hpp` 移到 `src/app/app_params.hpp`，仍属宿主侧 `pure_uav::app`；
- app 侧用 `flight::` 前缀引用 guidance；CMake include 根增加 `src/app`。

### 宿主侧改名（本次）

- `src/app/startup/application.{hpp,cpp}` → `src/app/guidance_application.{hpp,cpp}`；
- `src/app/tools/tools.{hpp,cpp}` → `src/app/guidance_sim_tools.{hpp,cpp}`；
- `src/app/app_params.hpp` → `src/app/guidance_app_params.hpp`；
- `Application` → `GuidanceApplication`，`AppParams` → `GuidanceAppParams`，`ParamResult` → `GuidanceParamResult`；
- tools 公开类型统一 `Guidance*` 前缀（`GuidanceCsvRow` / `GuidanceSyntheticWorld` / `GuidanceSimOptions` 等）；
- CMake target：`app_tools` → `guidance_sim_tools`，`app_startup` → `guidance_application`，删除 `app_guidance` 别名。

### 适配层 target 改名（本次）

- CMake target `state_adapter` → `guidance_state_adapter`；
- `guidance_shell` / `guidance_flight` / `test_state_adapter` 的链接同步更新；
- 文件名 `state_adapter.{hpp,cpp}` 与命名空间 `guidance::adapter` 不变。

### 移除 guidance_libs 兼容聚合（本次）

- 删除 INTERFACE target `guidance_libs`（= core + sim 的旧聚合名）；
- `guidance_libs/` 作为源码目录保留，不再作为 target；
- 仿真/测试直接链接 `guidance_core` + `guidance_sim`，机载链接 `guidance_flight`。

### 文件与公共类型改名（本次）

- `state_adapter.{hpp,cpp}` → `guidance_state_adapter.{hpp,cpp}`；
- `state_check.{hpp,cpp}` → `guidance_state_check.{hpp,cpp}`；
- `flight_types.hpp` → `guidance_flight_types.hpp`；
- `guidance::flight` / `guidance::adapter` 的公共类型统一 `Guidance*` 前缀：
  `GuidanceInsSample` / `GuidanceTargetSample` / `GuidanceCameraModel` /
  `GuidanceLoopConfig` / `GuidanceInputHealth` / `GuidanceControlMode` /
  `GuidanceLoopStatus` / `GuidanceInsSource` / `GuidanceTargetSource` /
  `GuidanceCommandSink` / `GuidanceNullSink` / `GuidanceLatencyStats` /
  `GuidanceLoopStats` / `GuidanceRingRecord` / `GuidanceRingLog` /
  `GuidanceFrame` / `GuidanceAttitudeFrame` / `GuidanceTimeUnit` /
  `GuidanceTimeBase` / `GuidanceStateTimeTracker` / `GuidanceInsRaw` /
  `GuidanceTargetRaw` / `GuidanceRawFrame` / `GuidanceSeverity` /
  `GuidanceCheckCode` / `GuidanceIssue` / `GuidanceReport`。

### app 顶层只保留 main.cpp（本次）

- `guidance_app_params.hpp` → `startup/guidance_app_params.hpp`；
- `guidance_application.{hpp,cpp}` → `startup/guidance_application.{hpp,cpp}`；
- `guidance_sim_tools.{hpp,cpp}` → `sim/guidance_sim_tools.{hpp,cpp}`；
- include 改为路径化：`startup/...` / `sim/...`；
- `src/app` 只保留 `startup/` 和 `sim/` 两个功能目录（`config/` 仅一个头文件，已合并到 `startup/`）；
- CMake 源文件路径同步；`guidance_sim_tools` / `guidance_application` 只以 `src/app` 为 include 根。

## 2026-10-25

- 反转距/参数/黑匣子基线：`test_guidance` 304 通过；
- `src/app/README.md` 记录 PPN N=4 a_max=10 的闭环基线。
