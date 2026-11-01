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

## 2026-10-25

- 反转距/参数/黑匣子基线：`test_guidance` 304 通过；
- `src/app/README.md` 记录 PPN N=4 a_max=10 的闭环基线。
