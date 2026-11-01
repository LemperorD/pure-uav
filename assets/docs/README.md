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
| 系统架构 / 数据流 / 线程模型 | [architecture.md](architecture.md) |
| 坐标、姿态、时间、单位、端口 | [interfaces.md](interfaces.md) |
| 测试矩阵 / 基线 / 怎么跑 | [testing.md](testing.md) |
| CSV 格式 / 零阶保持 / 回放边界 | [app/csv-format.md](app/csv-format.md) |
| 设计决策（为什么这么定） | [decisions/0001-quat-semantics.md](decisions/0001-quat-semantics.md)、[0002-pure-prefix.md](decisions/0002-pure-prefix.md)、[0003-los-rate-source.md](decisions/0003-los-rate-source.md) |
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
| `src/app/main.cpp` | [app/README.md](app/README.md) | 入口：只创建 `Application` |
| `src/app/startup/` | [app/README.md](app/README.md) | CLI / 参数加载 / 分发（`usage.md` 计划中） |
| `src/app/tools/` | [app/README.md](app/README.md) + [app/csv-format.md](app/csv-format.md) | CSV 录制/回放 + 合成世界（`blackbox.md` 计划中） |
| `src/guidance/flight/` | [guidance/README.md](guidance/README.md) + [architecture.md](architecture.md) | 飞行外壳：`flight_types` + `guidance_loop` |
| `src/guidance/guidance_libs/` | [guidance/README.md](guidance/README.md) + [decisions/0001](decisions/0001-quat-semantics.md) + [0002](decisions/0002-pure-prefix.md) | 算法核：`guidance_step` / `pn_law` / `quat` |
| `src/guidance/adapter/` | [guidance/README.md](guidance/README.md) + [interfaces.md](interfaces.md) | 坐标/时间适配 + 体检 |
| `src/guidance/sim/` | [guidance/README.md](guidance/README.md) | 传感器墙 / 目标模型 / `pn_sim` |
| `src/test/` | [testing.md](testing.md) | 测试用例与基线 |
| `src/driver/` | [src/driver/README.md](../../src/driver/README.md) | Orbbec 相机；通信见 `communication.md` |
| `sim_xtdrone2/` | [sim_xtdrone2/README.md](../../sim_xtdrone2/README.md) | 系统仿真接入（留原地） |
| `src/vio/` | [src/vio/README.md](../../src/vio/README.md) | 标定工具 |
| `src/common/` | 代码注释 | `vec3` / `mavlink_units` / `pure_math_tools` |

---

## 3 文档维护规则

1. **代码移动** → 只改第 2 节表格里的路径，不在代码里维护第二份对照表。
2. **接口变更** → 同步 `interfaces.md` 和对应的模块 README。
3. **跑法 / CLI 变更** → 同步 `app/README.md`（以后拆 `app/usage.md`）。
4. **设计原因 / 历史决策** → 放 `decisions/`（已有 0001–0003），不要塞回代码头。
5. **迁移/合并历史** → 放 `changelog.md` 或依赖 `git log`。
6. 文档只到模块级；具体文件用文件头 1 行反向链接指到本文档或对应模块文档。

---

## 4 计划中的文档（尚未创建）

以下文件**还没有**，先不要引用为已存在：

```text
assets/docs/comment-style.md         # 注释规范
assets/docs/app/usage.md             # CLI/YAML/三种跑法
assets/docs/app/blackbox.md          # RingLog / 黑匣子
assets/docs/decisions/               # 已有 0001–0003，后续继续增加
```

> 创建后，在上面的“按任务找”和“模块表”里补链接。
