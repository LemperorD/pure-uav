# 项目文档索引

> 代码在 `src/`，文档在 `assets/docs/`。
> 先按“你想做什么”找；找不到再看“代码模块 → 文档”。

---

## 1 按读者 / 任务找

| 你想做什么 | 先看 |
|---|---|
| 制导算法、接口、调参 | [guidance/README.md](guidance/README.md) |
| 制导排障、已知坑 | [guidance/后续注意问题.md](guidance/后续注意问题.md) |
| 制导任务分工 | [guidance/任务书.md](guidance/任务书.md) |
| 跑机载主循环、录制、回放 | [app/README.md](app/README.md) |
| CLI / YAML / 三种跑法 | [app/usage.md](app/usage.md) |
| 黑匣子（GuidanceRingLog / dump） | [app/blackbox.md](app/blackbox.md) |
| 注释规范 | [comment-style.md](comment-style.md) |
| 系统架构 / 数据流 / 线程模型 | [architecture.md](architecture.md) |
| 坐标、姿态、时间、单位、端口 | [interfaces.md](interfaces.md) |
| 测试矩阵 / 基线 / 怎么跑 | [testing.md](testing.md) |
| CSV 格式 / 零阶保持 / 回放边界 | [app/csv-format.md](app/csv-format.md) |
| 设计决策（为什么这么定） | [0001 四元数](decisions/0001-quat-semantics.md) · [0002 Pure 前缀](decisions/0002-pure-prefix.md) · [0003 ω](decisions/0003-los-rate-source.md) · [0004 冻结](decisions/0004-freeze-criterion.md) · [0005 时间](decisions/0005-time-base.md) · [0006 相机/VIO](decisions/0006-camera-vio.md) · [0007 目标模型](decisions/0007-target-models.md) · [0008 传感器墙](decisions/0008-sensor-wall.md) |
| 迁移 / 合并历史 | [changelog.md](changelog.md) |
| 系统仿真（XTDrone2 / Gazebo） | [sim_xtdrone2/README.md](../../sim_xtdrone2/README.md) |
| 通信任务书 | [communication.md](communication.md) |
| 目标检测任务书 | [detection.md](detection.md) |
| VIO 任务书 | [vio.md](vio.md) |
| 香橙派接入 | [orange-pi-zero-3w连接方式.md](orange-pi-zero-3w连接方式.md) |
| Orbbec 相机 | [src/driver/README.md](../../src/driver/README.md) |

---

## 2 代码模块 → 文档

只到**模块级**，不逐文件；代码移动时只改这张表。

| 代码位置 | 主要文档 | 说明 |
|---|---|---|
| `src/app/main.cpp` | [app/README.md](app/README.md) | 通信示例入口 |
| `src/app/startup/guidance_example.cpp` | [app/README.md](app/README.md) + [app/usage.md](app/usage.md) | 制导调用示例入口 |
| `src/app/startup/guidance_application.{hpp,cpp}` | [app/README.md](app/README.md) + [app/usage.md](app/usage.md) | CLI / 参数加载 / 分发 |
| `src/app/sim/guidance_sim_tools.{hpp,cpp}` | [app/README.md](app/README.md) + [app/csv-format.md](app/csv-format.md) + [app/blackbox.md](app/blackbox.md) | CSV 录制/回放 + 合成世界 + 黑匣子 |
| `src/guidance/flight/` | [guidance/README.md](guidance/README.md) + [architecture.md](architecture.md) | 飞行外壳：`flight_types` + `guidance_loop` |
| `src/guidance/guidance_libs/` | [guidance/README.md](guidance/README.md) + [0001](decisions/0001-quat-semantics.md) + [0002](decisions/0002-pure-prefix.md) + [0004](decisions/0004-freeze-criterion.md) + [0007](decisions/0007-target-models.md) + [0008](decisions/0008-sensor-wall.md) | 算法核 / 冻结 / 目标模型 / 传感器墙 |
| `src/guidance/adapter/` | [guidance/README.md](guidance/README.md) + [interfaces.md](interfaces.md) + [0005](decisions/0005-time-base.md) | 坐标/时间适配 + 体检 |
| `src/guidance/sim/` | [guidance/README.md](guidance/README.md) + [0007](decisions/0007-target-models.md) + [0008](decisions/0008-sensor-wall.md) | 传感器墙 / 目标模型 / `pn_sim` |
| `src/test/` | [testing.md](testing.md) | 测试用例与基线 |
| `src/driver/` | [src/driver/README.md](../../src/driver/README.md) | Orbbec 相机；通信见 `communication.md` |
| `sim_xtdrone2/` | [sim_xtdrone2/README.md](../../sim_xtdrone2/README.md) | 系统仿真接入（留原地） |
| `src/vio/` | [src/vio/README.md](../../src/vio/README.md) | 标定工具 |
| `src/common/` | 代码注释 | `vec3` / `mavlink_units` / `pure_math_tools` |

---

## 3 文档维护规则

1. **代码移动** → 只改第 2 节表格里的路径，不在代码里维护第二份对照表。
2. **接口变更** → 同步 `interfaces.md` 和对应的模块 README。
3. **跑法 / CLI 变更** → 同步 `app/usage.md`。
4. **设计原因 / 历史决策** → 放 `decisions/`（已有 0001–0008），不要塞回代码头。
5. **迁移/合并历史** → 放 `changelog.md` 或依赖 `git log`。
6. 文档只到模块级；具体文件用文件头 1 行反向链接指到本文档或对应模块文档。

---

## 4 维护状态

2026-11-01：原计划的文档（architecture / interfaces / testing / comment-style / app/usage / app/blackbox / decisions 0001–0008）均已创建。
后续新增（如 `flight_logger`、Doxygen）再补索引。
