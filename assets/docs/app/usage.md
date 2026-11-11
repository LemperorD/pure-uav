# app 用法（CLI / YAML / 跑法）

- 状态：现行
- 相关代码：`src/app/main.cpp`（机载组合根）、`src/app/startup/app_options.cpp`（参数解析）、`src/app/runtime/vio_pose_publisher.cpp`（位姿发布）、`src/app/startup/guidance_example.cpp`（制导示例入口）、`src/app/startup/guidance_application.{hpp,cpp}`
- 相关文档：[README.md](README.md)、[csv-format.md](csv-format.md)、[blackbox.md](blackbox.md)

---

## 1 机载入口与线程

`main.cpp` 保留信号处理、模块启动顺序、固定节拍公共循环和退出清理，
便于直接阅读通信、VIO、检测和任务之间的调用关系。
命令行解析与帮助移到 `startup/app_options.cpp`，
配置合成、飞控检查和任务/检测启动细节移到 `startup/app_startup.cpp`；
VIO 发布门控、计数和状态日志移到 `runtime/vio_pose_publisher.cpp`。
`main()` 直接调用这些函数并持有模块对象，公共循环也在 `main()` 内。

`./bin/main` 连接 PX4，运行通信、VIO 与检测子进程监督；相机由先行启动的 `camera_server` 独占。
主入口和任务适配器统一接受 PX4；其他固件会在连接相机前被拒绝。
`--check-config` 会打印 `flight_stack: PX4`。外部定位融合验收见
[uav-vio-bench-checklist.md](uav-vio-bench-checklist.md)。
`--mission` 显式启用主任务，通过 MissionService / MissionRuntime 调度 Offboard、
解锁、起飞、目标搜索、制导和返航；接线及当前能力边界见
[任务文档](../../../src/app/mission/README.md)。

| 线程 / 服务 | 职责 | 数据交付 |
|---|---|---|
| 通信接收（Communicator 内部） | 唯一串口、心跳、IMU、遥测 | 逐帧 IMU 队列、遥测缓存 |
| camera_server（独立进程） | 唯一相机采集 | VIO 灰度/深度通道、检测彩色通道 |
| VIO（main 内部） | 顺序处理 IMU/图像、健康监控 | 位姿快照，经同一串口发送 ODOMETRY |
| detect_rt（子进程） | 彩色通道取帧、模型推理 | JSON / 检测预览；任务开启时独立通道回传目标 |
| 检测监督线程（main 内部） | 启动后监督、异常重启、退出回收 | 完成状态交给主循环 |
| main | 位姿发布门控、输入监督、任务服务调用、统一退出 | Ctrl-C / SIGTERM / 故障处理；SIGUSR1 请求返航 |

```bash
# 从仓库根目录执行；相机独立终端启动：
./bin/camera_server --config config/camera.yaml
# 另一个终端：运行通信、VIO 与检测
./bin/main --no-send
# 仅 VIO 台架验收
./bin/main --no-send --no-detection
# 应用与 VIO 配置校验，不打开任何设备或启动子进程
./bin/main --check-config
# 检测配置由检测进程解析；运行参数逐个透传
./bin/main --detection-config config/detector-rt.yaml \
  --detect-arg --shm --detect-arg /pure_uav_camera_color
./bin/main --help
```

`--detect-bin FILE` 覆盖检测子程序路径；默认使用 `main` 同目录下的 `detect_rt`。
`--detection-config FILE` 传给子进程 `--config`；模型、阈值、类别、限频和帧龄
统一由 `config/detector-rt.yaml` 管理，不接受 `main --model` 覆盖。
`--mission-config` 指向任务配置，默认 `config/mission.yaml`；
`--guidance-config` 在任务启用时加载制导及彩色相机标定参数。
`--mission --no-send --check-config` 可校验实际任务配置，不打开设备。
`--no-send` 也禁止任务 setpoint 和飞控动作请求，预热不会假装成功。
`--blackbox` 仍是预留参数。

检测异常退出最多自动重启 3 次；监督循环不阻塞 VIO。检测正常结束或重启超限后，
主循环退出；相机断供、VIO 故障和用户停止时同样回收检测子进程，再关闭 VIO 与串口。
相机服务由外部托管，不随 `main` 关闭。

`main` 与 `vio_live` 是替代入口，不可同时占用同一飞控串口。
相机默认 1920×1080@30，必须匹配 VIO 标定；采集、曝光和增益在 `config/camera.yaml` 中设置。
`main` 通过 `config/vio.yaml` 决定是否使用深度，当前启用深度初始化。
完整协议、尺寸/深度校验和时间戳约定见 [camera-ipc.md](camera-ipc.md)。

## VIO 配置

`main` 从 `config/app.yaml` 加载应用调度参数和配置路径，再读取 `config/vio.yaml`。
检测算法使用独立的 `config/detector-rt.yaml`，由检测子进程解析。

| 文件 | VIO 侧的用途 |
|---|---|
| `config/app.yaml` | 保留应用调度设置，通过 `vio_config` 指向 VIO 参数文件 |
| `config/vio.yaml` | 标定目录、期望分辨率/帧率、IMU 频率、初始化选项、共享内存连接和位姿发布门限 |
| `config/driver.yaml` | 沿用原有通信配置；加载时校验 IMU 订阅周期与 VIO 的 `imu_hz` 一致 |

`config/vio.yaml` 的 `config_dir` 指向原有 OpenVINS 参数和相机/IMU 标定链。
`use_depth` 表示 VIO 是否使用深度图；相机采集与曝光等设置由 `camera_server` 管理。
原 `app.yaml` 中的 `use_depth_init: true` 和 `init_vel_zero: true` 已迁入 `vio.yaml`，
现在会用于 `main`。独立调试入口 `vio_live` 继续使用自身命令行参数。

应用配置及其中的相对路径均相对于仓库根目录解析。以下检查不打开串口或相机：

```bash
./bin/main --check-config
./bin/main --vio-config config/vio.yaml --no-send --check-config
```

`--config` 选择应用配置，`--vio-config` 选择 VIO 运行配置；
`--config-dir` 覆盖 VIO 标定目录并读取其分辨率；
`--shm-vio` 和 `--shm-timeout-ms` 覆盖 VIO 的共享内存连接参数。

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

`main.cpp` 组合通信、VIO、检测监督与 `MissionService`。
`glue/mission_ports.cpp` 读取飞控位置/姿态，`glue/mission_service.cpp` 接收检测结果并通过
`FcuClock` 将采集单调时钟映射到飞控 boot 时钟，任务运行层统一发送 `GuidanceCmd`。
VIO 的 LOCAL_FRD 位姿只发给飞控；制导读取飞控融合后的 LOCAL_POSITION_NED 与姿态，避免混用坐标系。
独立 `GuidanceApplication` 仍负责仿真/回放。
