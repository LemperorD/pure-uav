# app 用法（CLI / YAML / 跑法）

- 状态：现行
- 相关代码：`src/app/main.cpp`（并发机载主流程）、`src/app/startup/guidance_example.cpp`（制导示例入口）、`src/app/startup/guidance_application.{hpp,cpp}`
- 相关文档：[README.md](README.md)、[csv-format.md](csv-format.md)、[blackbox.md](blackbox.md)

---

## 1 机载入口与线程

`./bin/main` 直接启动机载主流程，所有功能同时运行，不使用子命令。

| 线程 | 职责 | 数据交付 |
|---|---|---|
| 通信接收（Communicator 内部） | 唯一串口、心跳、IMU、位置和姿态 | IMU 有界队列、线程安全遥测缓存 |
| 相机采集（OrbbecCamera 内部） | 唯一相机，彩色解码一次并生成灰度 | 灰度送 VIO，BGR 送检测 |
| 检测推理（Detector 内部） | 只处理最新彩色帧，不自行打开相机 | 最新检测框 + 原始采集时刻 |
| VIO | 顺序喂 IMU/图像、健康监控、输出位姿 | 通过同一个 Communicator 发送 ODOMETRY |
| 制导 | 默认 50 Hz 读取遥测与检测结果，运行 GuidanceLoop | 通过同一个 Communicator 发送控制指令 |
| main | 启动、监督、状态输出、停止与 join | Ctrl-C / SIGTERM / 故障触发统一退出 |

```bash
# 从仓库根目录执行；默认同时启动所有机载功能
./bin/main
# 台架检查：运行计算，不发送 ODOMETRY 或制导指令
./bin/main --no-send
# 指定各模块配置，并在退出时保存黑匣子
./bin/main --driver-config config/driver.yaml \
  --config-dir src/vio/estimator/config/gemini_1920x1080 \
  --guidance-config config/onboard-guidance.yaml --blackbox flight_bb.csv
./bin/main --help
```

`--model FILE` 覆盖检测模型；默认按编译后端选择 `yolo26n.onnx` / `yolo26n.nb`。
`main` 必须同时具备实时 VIO 和检测依赖；模块缺失时启动失败，不以部分功能冒充整机运行。
原有 `detect_rt`、`vio_live`、`vio_offline`、`guidance_example` 保留供独立调试。

启动时加载模型和 VIO 标定，再打开一个串口、一个相机。主循环不自动解锁、起飞或切换飞行模式。
IMU 请求频率为 200 Hz；相机分辨率来自 VIO 标定，彩色 30 Hz，主流程不启用深度流。
当前安装假设 IMU 系与机体系重合，`T_imu_cam` 旋转直接作为相机到机体外参；实际安装需与标定一致。
检测像素先按标定去畸变，再交给制导。检测没有距离和目标速度，机载配置使用
**PPN + FROM_LOS_DIFFERENCE**，`t_go_freeze=0` 关闭依赖距离的冻结；不接受需要测距的 TPN/APN 配置。

VIO 图像队列最多 4 帧，溢出丢最旧帧并显示丢帧数；IMU 队列最多 2000 帧。
连接会话变化、IMU 丢帧或时间基跳变会重建 VIO，丢弃旧图像并复位制导。
串口初始等待最多 10 秒；相机断供 3 秒、IMU 断供 3 秒（启动宽限 10 秒）、
检测断供 10 秒、制导数据超时或发送失败均触发停止。
退出时先通知并 join 算法线程，再停止相机投递、检测线程，最后关闭通信。

## 1.1 独立制导仿真入口

```text
bin/guidance_example
  → GuidanceApplication::run(argc, argv)
      → 默认值 + YAML + CLI → GuidanceAppParams
      → guidance_validate_params()
      → 合成世界 / CSV 回放
```

以下 CLI、YAML 及退出码说明针对 `guidance_example`。所有命令从仓库根目录执行。

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

## 10 机载接线

`main.cpp` 中的 `InsSource` 使用飞控位置/姿态，`TargetSource` 消费检测最新结果并通过
`FcuClock` 将采集单调时钟映射到飞控 boot 时钟，`CommandSink` 转成通信模块的 `GuidanceCmd`。
VIO 的 LOCAL_FRD 位姿只发给飞控；制导读取飞控融合后的 LOCAL_POSITION_NED 与姿态，避免混用坐标系。
独立 `GuidanceApplication` 仍负责仿真/回放。
