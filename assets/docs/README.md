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
| `src/app/tools/` | [app/README.md](app/README.md) | CSV 录制/回放 + 合成世界（`csv-format.md`、`blackbox.md` 计划中） |
| `src/guidance/flight/` | [guidance/README.md](guidance/README.md) + [app/README.md](app/README.md) | 飞行外壳：`flight_types` + `guidance_loop` |
| `src/guidance/guidance_libs/` | [guidance/README.md](guidance/README.md) | 算法核：`guidance_step` / `pn_law` / `quat` |
| `src/guidance/adapter/` | [guidance/README.md](guidance/README.md) | 坐标/时间适配（`interfaces.md` 计划中） |
| `src/guidance/sim/` | [guidance/README.md](guidance/README.md) | 传感器墙 / 目标模型 / `pn_sim` |
| `src/test/` | 暂无独立文档 | 测试用例与基线（`testing.md` 计划中） |
| `src/driver/` | [src/driver/README.md](../../src/driver/README.md) | Orbbec 相机；通信见 `communication.md` |
| `sim_xtdrone2/` | [sim_xtdrone2/README.md](../../sim_xtdrone2/README.md) | 系统仿真接入（留原地） |
| `src/vio/` | [src/vio/README.md](../../src/vio/README.md) | 标定工具 |
| `src/common/` | 代码注释 | `vec3` / `mavlink_units` / `pure_math_tools` |

---

## 3 文档维护规则

1. **代码移动** → 只改第 2 节表格里的路径，不在代码里维护第二份对照表。
2. **接口变更** → 同步 `interfaces.md`（计划中）和对应的模块 README。
3. **跑法 / CLI 变更** → 同步 `app/README.md`（以后拆 `app/usage.md`）。
4. **设计原因 / 历史决策** → 放 `decisions/`（计划中），不要塞回代码头。
5. **迁移/合并历史** → 放 `changelog.md`（计划中）或依赖 `git log`。
6. 文档只到模块级；具体文件用文件头 1 行反向链接指到本文档或对应模块文档。

---

## 4 计划中的文档（尚未创建）

以下文件**还没有**，先不要引用为已存在：

```text
assets/docs/architecture.md          # 跨模块分层/数据流/线程模型
assets/docs/interfaces.md            # 端口/坐标系/单位/时间戳/错误码
assets/docs/testing.md               # 304/1193/120 基线、怎么跑
assets/docs/changelog.md             # 迁移/合并历史
assets/docs/comment-style.md         # 注释规范
assets/docs/app/usage.md             # CLI/YAML/三种跑法
assets/docs/app/csv-format.md        # 录制/回放格式、零阶保持
assets/docs/app/blackbox.md          # RingLog / 黑匣子
assets/docs/decisions/               # ADR：为什么这么设计
```

> 创建后，在上面的“按任务找”和“模块表”里补链接。
