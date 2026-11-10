# app 用法（CLI / YAML / 跑法）

- 状态：现行
- 相关代码：`src/app/main.cpp`（并发机载主流程）、`src/app/startup/guidance_example.cpp`（制导示例入口）、`src/app/startup/guidance_application.{hpp,cpp}`
- 相关文档：[README.md](README.md)、[csv-format.md](csv-format.md)、[blackbox.md](blackbox.md)

---

## 1 机载入口与线程

`./bin/main` 是机载主流程入口，负责通信、检测、VIO、制导等模块的整机集成，不使用子命令。
本节保留整机集成的模块分工与运行约定；各入口的功能接入状态、参数生效范围见对应模块说明和 `--help`。
本次相机接口迁移只更新 VIO 取图路径，具体启动命令与适配范围见 [VIO 接入 camera_server](camera-ipc.md)。

| 线程 / 服务 | 职责 | 数据交付 |
|---|---|---|
| 通信接收（Communicator 内部） | 唯一串口、心跳、IMU、位置和姿态 | IMU 有界队列、线程安全遥测缓存 |
| 相机服务（camera_server，独立进程） | 统一持有相机，采集并发布灰度、深度及彩色帧 | VIO 通道与彩色检测通道分别提供共享内存数据 |
| 检测推理（Detector 内部） | 只处理最新彩色帧，不自行打开相机 | 最新检测框 + 原始采集时刻 |
| VIO | 从相机服务取图，顺序喂 IMU/图像、健康监控、输出位姿 | 通过同一个 Communicator 发送 ODOMETRY |
| 制导 | 默认 50 Hz 读取遥测与检测结果，运行 GuidanceLoop | 通过同一个 Communicator 发送控制指令 |
| main | 启动、监督、状态输出、停止与 join | Ctrl-C / SIGTERM / 故障触发统一退出 |

相机服务由部署方先行启动，VIO 消费者使用默认 `/pure_uav_camera_vio` 通道；
自定义通道与等待超时可通过 `--shm-vio NAME`、`--shm-timeout-ms N` 设置。
检测端需完成彩色通道接入后才能共用该服务，具体接入状态由检测模块说明维护。

```bash
# 从仓库根目录执行；先启动 camera_server
./bin/main
# VIO 台架检查：运行计算与发布门控，不发送 ODOMETRY
./bin/main --no-send
# 整机集成版的模块参数示例；参数生效范围以对应入口 --help 为准
./bin/main --driver-config config/driver.yaml \
  --config-dir src/vio/estimator/config/gemini_1920x1080 \
  --guidance-config config/onboard-guidance.yaml --blackbox flight_bb.csv
./bin/main --help
```

整机集成约定中，`--model FILE` 覆盖检测模型；默认按编译后端选择 `yolo26n.onnx` / `yolo26n.nb`。
完整整机集成要求同时具备实时 VIO 和检测依赖；单模块适配与测试结果不能替代整机集成验收。
原有 `detect_rt`、`vio_live`、`vio_offline`、`guidance_example` 保留供独立调试。

整机集成时加载模型和 VIO 标定，通信模块持有唯一串口，相机由 `camera_server` 统一采集。主循环不自动解锁、起飞或切换飞行模式。
IMU 请求频率为 200 Hz；相机服务的分辨率、帧率须匹配 VIO 标定与消费者期望，默认彩色 30 Hz。
曝光、增益、聚合和深度采集由服务端配置；消费者是否使用深度由各模块配置决定。
当前安装假设 IMU 系与机体系重合，`T_imu_cam` 旋转直接作为相机到机体外参；实际安装需与标定一致。
检测像素先按标定去畸变，再交给制导。检测没有距离和目标速度，机载配置使用
**PPN + FROM_LOS_DIFFERENCE**，`t_go_freeze=0` 关闭依赖距离的冻结；不接受需要测距的 TPN/APN 配置。

VIO 共享内存输入读取最新完整帧，跳过的帧计入丢帧数；原整机集成的 IMU 队列上限为 2000 帧。
连接会话变化、IMU 丢帧或时间基跳变会重建 VIO，丢弃旧图像并复位制导。
原整机集成约定为串口初始等待最多 10 秒、IMU 断供 3 秒（启动宽限 10 秒）、
检测断供 10 秒、制导数据超时或发送失败均触发停止。
退出时先通知并 join 算法线程，再停止输入与检测线程，最后关闭通信。
相机服务的超时检测、停止与重启约定见 [camera-ipc.md](camera-ipc.md)；停止 VIO 消费者不会停止相机服务。

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
