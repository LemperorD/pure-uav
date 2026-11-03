# app 用法（CLI / YAML / 跑法）

- 状态：现行
- 相关代码：`src/app/main.cpp`（通信示例）、`src/app/startup/guidance_example.cpp`（制导示例入口）、`src/app/startup/guidance_application.{hpp,cpp}`
- 相关文档：[README.md](README.md)、[csv-format.md](csv-format.md)、[blackbox.md](blackbox.md)

---

## 1 入口

```text
bin/main                        # 通信示例（全局 main，不跑制导）
bin/guidance_example            # 制导调用示例（独立可执行文件）
  → GuidanceApplication::run(argc, argv)
      → 默认值 + YAML + CLI → GuidanceAppParams
      → guidance_validate_params()
      → 合成世界 / 回放 / 未来真机
```

所有命令都从**仓库根目录**执行。

---

## 2 CLI

| 参数 | 默认 | 说明 |
|---|---|---|
| `--hz=N` | 50 | 主循环频率 |
| `--law=N` | 0 | 0=PPN，1=TPN，2=APN |
| `--n=X` | 4 | 导航比 |
| `--a-max=X` | 10 | 过载上限 m/s² |
| `--ticks=N` | 300 | 总拍数 |
| `--realtime` | 关 | 按真实时间节拍；默认全速 |
| `--quiet` | 关 | 只打汇总 |
| `--record=F` | — | 输入录制 CSV |
| `--out=F` | — | 指令落盘 CSV |
| `--replay=F` | — | 回放输入 CSV |
| `--config=F` | — | YAML 参数文件 |
| `--print-params` | 关 | 打印最终参数后退出 |
| `--blackbox=F` | — | 跑完写黑匣子 CSV |
| `--blackbox-sec=S` | 5.0 | 黑匣子窗口（秒；<=0 全部） |
| `--help` / `-h` | — | 帮助 |

---

## 3 优先级

```text
出厂默认值  <  YAML 文件  <  显式命令行
```

- 命令行**没给**的项不会覆盖 YAML；
- 只有 `--law / --n / --a-max / --hz` 支持命令行覆盖；
- `--ticks / --record / --out / --replay / --blackbox` 是运行选项，不进 `GuidanceAppParams`。

---

## 4 YAML 子集

支持：

- 嵌套：
  ```yaml
  guidance:
    law: PPN
    N: 4
  ```
- 扁平点号：`guidance.N: 4`
- `#` 注释（引号外）、空行、字符串两端引号；
- 空格缩进；**tab 直接报错**；
- 重复键/未知键/类型错 → 报 `文件:行号: 原因`。

不支持：列表、锚点、多文档、行内映射。

---

## 5 支持的键

| 段 | 键 | 默认 |
|---|---|---|
| `guidance.` | `law` | `PPN` |
| | `N` | 4.0 |
| | `a_max` | 10.0 |
| | `freeze_criterion` | `BY_TGO` |
| | `t_go_freeze` | 0.15 s |
| | `R_freeze` | 5.0 m |
| | `los_rate_source` | `FROM_RELATIVE_VELOCITY` |
| `loop.` | `loop_hz` | 50 |
| | `max_sample_age_s` | 0.2 |
| | `sensor_timeout_s` | 0.5 |
| | `on_unhealthy` | `HOLD_LAST_COMMAND` |
| | `hold_last_max_s` | 0.2 |
| | `full_check_hz` | 2.0 |
| | `send_on_every_tick` | true |
| `cam.` | `fx` / `fy` | 0（未配） |
| | `cx` / `cy` | 0 |
| | `R_body_cam` | 单位阵 |

枚举名大小写不敏感，也接受数字（如 `law: 2`）。

---

## 6 校验

- `N ∈ (0.5, 20]`；
- `a_max ∈ (0, 100]`；
- `loop_hz ∈ [1, 1000]`；
- `sensor_timeout_s ≥ max_sample_age_s`；
- `hold_last_max_s ≤ sensor_timeout_s`；
- `cam.fx / cam.fy` 必须同时配、不能为负；
- `R_body_cam` 必须 9 个有限数。

失败返回码 **2**。

---

## 7 相机

- 配置里没给相机 → **只有合成世界 demo** 才回退 `guidance_demo_camera()`；
- 真机必须在 YAML 里给 `cam.fx/fy/cx/cy/R_body_cam`，否则反投影会用假内参；
- 真机外参来自 VIO 标定，见 [decisions/0006-camera-vio.md](../decisions/0006-camera-vio.md)。

---

## 8 跑法

```bash
# ① 合成世界闭环（零依赖）
./bin/guidance_example --ticks=300

# ② 边跑边录
./bin/guidance_example --ticks=300 --record=flight.csv --out=cmd.csv

# ③ 回放
./bin/guidance_example --replay=flight.csv --ticks=300 --out=replay_cmd.csv

# ④ 参数文件
./bin/guidance_example --config=guidance.yaml --print-params

# ⑤ 黑匣子
./bin/guidance_example --ticks=300 --blackbox=bb.csv --blackbox-sec=10
```

---

## 9 退出码

| 码 | 含义 |
|---|---|
| 0 | 成功 / help / print-params |
| 1 | 运行失败（打不开文件、init 失败、sink 错误等） |
| 2 | 命令行/配置/校验失败 |

---

## 10 真机（未来）

真机模式会构造真实源/出口 + `GuidanceLoop`：

- `MavlinkInsSource` / `VisionTargetSource`；
- `MavlinkAccelSink`；
- 胶水放 `src/app/glue/`，由 `main` 链接 `communicate`。

当前 `GuidanceApplication` 仍只分发合成世界/回放。
