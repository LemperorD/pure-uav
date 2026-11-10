# 相机服务与跨进程帧分发

日期：2026-11-09。本文描述当前工作树的**实际运行架构**：一台 Orbbec 相机只被初始化一次，
VIO 与检测作为独立进程从共享内存取帧。

历史背景：此前 `detect_rt`（`CameraSource`）、`vio_live`（`OrbbecCamera`）与 `bin/main`（`Detector`）
各自 `new ob::Pipeline()` 并 `start()`。Orbbec SDK 设备独占，任意两个同时运行必有一个打不开相机。
`assets/docs/app/single-process-vio.md` / `detection-integration.md` 描述的是外部工作副本中的
「单进程共享」设计，与本树实现不同；按用户确认，本树采用多进程 + 进程隔离方案。

## 1. 架构

```text
                      ┌─────────────────────────────────────────┐
 Orbbec 设备 ──唯一──▶ │ camera_server（唯一 ob::Pipeline）        │
                      │  OrbbecCamera::startWithFrames()         │
                      └──────────┬──────────────┬───────────────┘
                                 │              │
              VIO 通道 /pure_uav_camera_vio      │ 彩色通道 /pure_uav_camera_color
              gray + depth_mm + 元数据           │ 原始编码字节 + 元数据
                                 │              │
                                 ▼              ▼
                        ┌────────────────┐  ┌────────────────────┐
 飞控 HIGHRES_IMU ─────▶│ vio_live       │  │ detect_rt          │
 200 Hz（串口独占）      │ FcuClock/VIO   │  │ DetectionRuntime   │
                        └───────┬────────┘  └─────────┬──────────┘
                                ▼                     ▼
                     MAVLink ODOMETRY        JSON / 带框 frame.jpg
```

`camera_server` 与 `vio_live` 由外部托管启动（systemd / 板端自启动 / 现场脚本）；
**检测子进程由 `bin/main` 等飞控心跳后 fork/exec 拉起并监督**（信号转发、异常最多自动重启 3 次、
退出时有序停止）。仓库不提供统一启动脚本。
外部托管方先清理残留共享内存并启动 `camera_server` / `vio_live`，再运行 `bin/main`；
退出时先停 `bin/main`（它负责停检测子进程），再停 VIO 与相机服务。
`scripts/start_detection_stream.sh` 只负责 mjpg-streamer 前端。

`main` 只做进程编排：不解析 `config/detector-rt.yaml`、不实例化 `DetectionRuntime`、
不连接共享内存、不参与取帧与推理；检测算法参数仍由 `bin/detect_rt` 自行读取。

进程隔离：检测进程原生 NPU 崩溃或挂起不会拖垮 VIO，由 `bin/main` 自动重启 `detect_rt`
（最多 3 次，超限 `main` 退出交由外部托管方处置）；`vio_live` 与 `camera_server` 无需重启。

## 2. 帧通道

代码：`src/common/camera_frame.hpp`（协议 POD）、`src/common/camera_shm.hpp/.cpp`（实现）、
`src/camera/frame_publisher.*`（服务端拆分发布）。

单通道布局：

```text
[0, kHeaderBytes)                                  ShmHeader：magic/version/layout/StreamMeta/原子状态
[kHeaderBytes + i*slot_stride, +kSlotHeaderBytes)  SlotHeader：seqlock + FrameMeta
[+kSlotHeaderBytes, +slot_payload_bytes)           负载：gray | depth | color（三段连续）
```

- **两条独立通道**：VIO 每帧约 6 MB（1920x1080 灰度 2 MB + CV_16U 深度 4 MB，30 fps 约 180 MB/s），
  检测只需几百 KB 的 MJPG 且限频 10 Hz。分开后检测侧慢或挂起不占用 VIO 槽位。
- **seqlock + 三缓冲**：写者把槽序号置奇数、写入、置偶数；读者记录前后序号并拷到自有缓冲，
  序号变化或为奇数则重试。写者永不阻塞，读者不读撕裂帧，天然「丢最旧」。
  **不用 `PTHREAD_PROCESS_SHARED` 互斥锁**：持有者崩溃会永久死锁，直接违背进程隔离目标。
- 读者打开时校验 magic / version / header_bytes / slot_count / slot_stride / total_bytes 与通道类型，
  并校验每帧三段长度之和不超过槽容量，防越界。
- 共享内存权限 `0660`；`shm_unlink` 在 `ShmChannelWriter::close()` 与写者进程退出时都会执行；
  外部托管方在启动前应清理残留通道对象。
- VIO 通道默认 3×8 MiB，彩色通道默认 3×4 MiB（见 `config/camera.yaml`）。`/dev/shm` 容量需 ≥ 40 MiB。

## 3. 时间与帧标识

| 字段 | 含义 |
|---|---|
| `seq` | 从 1 起单调递增，两个通道取同一帧集编号，供消费者关联与去重 |
| `host_ns` | `CLOCK_MONOTONIC`（与 `std::chrono::steady_clock` 同源），跨进程可比；采集时刻 |
| `device_us` | SDK `timeStampUs()` 原值；时基不同、增量并非微秒，**只能当相对量** |
| `sys_us` | SDK `systemTimeStampUs()`，主机时基；VIO `--cam-ts device` 沿用此语义 |
| `depth_device_us` | SDK 深度帧 `timeStampUs()`，与 `device_us` 同源，用于深度年龄判断 |
| `color_encoding` | `ColorEncoding`（jpeg / rgb / yuyv / bgr），逐帧写在 `FrameMeta` |

- VIO 的 `FcuClock` 线性回归、`ImuTimestampFilter`、`imuCoversImage`、`--cam-ts`、
  `cameraTimeOffsetSec()`、`feedImu/feedImage`、`sendVisionPose` 全部保持不变。
  消费者把 `FrameItem.device_us` 填为 `sys_us`，与原 `OrbbecCamera::FrameCallback` 语义一致。
- 检测侧 `DetectionRuntime` 把 `FrameResult.host_us` 置 0（单调时钟不能冒充 Unix/飞控时间），
  JSON 输出改带 `host_ns`（`TargetSnapshot.captured_host_ns`）。需要飞控 boot 时间时
  仍须按 `assets/docs/interfaces.md` §4.3 用同帧 `SYSTEM_TIME` 单向换算。
- `StreamMeta` 携带深度内参与彩色→深度外参（`fx/fy/cx/cy/R_c2d/t_mm/depth_width/depth_height`），
  消费者不再向驱动查询 `depthInfo()`。`--record` 的 `depth_meta.txt` 也来自这份元数据。

## 4. 会话与失效

- `camera_server` 启动时 `setSession(1, 0)`。相机服务重启后共享内存对象被重建，
  消费者读到的 `session/generation` 变化会：
  - `detect_rt` 调 `DetectionRuntime::setEpoch()` 清空 pending 与旧结果；
  - `vio_live` 的 `ShmChannelReader` 重置 `last_seq_`，允许从新会话的最小序号接收。
- 心跳 `heartbeat_ns` 由写者每帧更新。`vio_live` 2 s 无心跳退出（由外部托管方重新拉起），
  `detect_rt` 3 s 无心跳退出（由 `bin/main` 监督重启，最多 3 次）；
  慢消费者只丢自己的旧帧，不影响采集。
- 彩色通道在服务端按 `color_max_hz`（默认 10）限频；`DetectionRuntime` 的
  `max_hz` / `max_age_ms` / `seq` 单调校验保留为二道防线。两处限频是**独立门限**：
  前者在 `config/camera.yaml` 的 `channels.color_max_hz`，控制服务端写入共享内存的频率；
  后者在 `config/detector-rt.yaml` 的 `max_hz`，控制检测侧接受帧与判旧帧龄的速率。
  本次没有合并这两处配置，调参时需分别确认。

## 5. 构建与运行

```bash
cmake -S . -B build-uav -DCMAKE_BUILD_TYPE=Release
cmake --build build-uav -j2

./bin/camera_server --check-config                 # 不打开相机，只校验 config/camera.yaml
./bin/detect_rt --check-config                     # 不加载模型，只校验 config/detector-rt.yaml

# 相机服务与 VIO 由外部托管启动：
./bin/camera_server --config config/camera.yaml
./bin/vio_live --shm /pure_uav_camera_vio --no-send

# 检测由 bin/main 等飞控心跳后拉起并监督：
./bin/main --detection-config config/detector-rt.yaml
./bin/main --detect-arg --json --detect-arg --frames --detect-arg 100  # 透传运行参数
./bin/main --no-detection                         # 只跑通信，不拉起检测
./scripts/start_detection_stream.sh --replace      # 另一个终端：mjpg-streamer 前端
```

`config/camera.yaml` 是相机参数唯一真源（分辨率/帧率/深度/曝光/测光/通道名/槽大小/彩色限频）。
`vio_live` 的 `--width/--height/--fps` 与曝光类 CLI 只用于兼容旧脚本，不再控制硬件，
启动时若与通道元数据不一致会打印警告；VIO 标定始终以 camera_server 配置为准。

**`config/detector-rt.yaml` 是检测算法参数唯一真源**：顶层键 `detection-rt:`，
9 个必填字段为 `model_path` / `imgsz` / `num_classes` / `conf_thr` / `iou_thr` /
`max_det` / `target_class` / `max_hz` / `max_age_ms`，与 `RuntimeConfig` 一一对应。
`model_path: auto` 按编译后端自动选择 `assets/models/yolo26n.onnx`(CPU) 或
`.nb`(NPU)；也可写显式路径。字段缺失、未知字段、旧占位字段（`model`、
`confidence_threshold` 等）、越界值或模型文件不存在都会明确报错并退出，不做兼容别名。
`detect_rt` 不提供 `--model/--conf/--iou` 等对应命令行选项，`--config FILE` 只改配置路径；
实时模式与 `--image FILE` 离线单图模式共用同一份 `RuntimeConfig` 与 `RuntimeBackend`。
改完配置需重启 `bin/main`（或单独重启 `detect_rt`）生效；`main` 不解析、不缓存这份配置。
`main` 的 `--detection-config FILE` 只改配置文件路径，`--detect-arg ARG` 只透传运行参数，
均不覆盖任何检测算法参数。

## 6. 互斥清单（禁止与 camera_server 同时运行）

| 入口 | 处理 |
|---|---|
| `bin/main` | 等飞控心跳后 fork/exec 拉起并监督 `bin/detect_rt`，自身不打开相机、不解析检测配置、不连共享内存 |
| `bin/detect_rt` / `bin/vio_live` | 已是只读消费者，不再打开设备 |
| `Detector` / `CameraSource` | 遗留源码保留给既有测试；`external_camera=true` 时不再打开相机 |
| `src/app/camera_service.cpp` | 未接线的单进程设计代码，保留作语义参考，不要接入构建 |
| `bin/test/test_camera` | 手工直采工具，需 `PURE_UAV_ALLOW_DIRECT_CAMERA=1` 显式确认 |
| `src/vio/estimator/tools/depth_range_probe.cpp` | 未注册 CMake；若补构建会直接调 `OrbbecCamera::start()`，禁止与相机服务并存 |
| `src/vio/intrinsic/**`、`src/vio/extrinsic/capture/**` 的 V4L2 脚本 | 与 Orbbec SDK 抢同一 USB 设备，共享内存方案对它们无效，必须在停用相机服务后离线运行 |

## 7. 验证边界

- 本机（x86_64、无真实 Orbbec 设备与板端 NPU）已完成：
  - 全量 CMake 配置与构建（`camera_server` / `vio_live` / `detect_rt` / `main` / `test_camera_shm`）；
  - `ctest -R camera_shm`：布局校验、跨进程 mmap 读取、seqlock 撕裂检测重试、seq 单调、
    会话切换失效、长度越界拒绝、双通道隔离、**两个消费者进程并发取帧**、清理；
  - `ctest -R detector_config`、`ctest -R detection_process`（后者用 `/bin/true` 与临时脚本
    覆盖子进程启动、回收、正常退出不重启、异常退出重启上限、停止请求、信号转发与参数透传）；
  - `camera_server --check-config`、两个消费者 `--help`、`main --help`、无相机服务时消费者明确拒绝启动；
  - 全量 `ctest`：23 项中 21 通过。`onboard_runtime` 随 `external_camera` 修复转为通过；
    `guidance` / `state_adapter` 是制导模块既有失败，本次未改动 `src/guidance/**`，与相机改造无关。
- 必须在板端/真机验证，**不得用本机结果代替**：
  - 真实 Orbbec 采集 + 深度对齐、30 fps 下共享内存写带宽与 `/dev/shm` 容量；
  - 真实 `.nb` NPU 推理（`vip_lite.h` / `libNBGlinker.so` / `libVIPhal.so`）与类别语义；
  - VIO 时间同步、IMU 融合、飞控 ODOMETRY 接收率与检测开启前后的定位质量对比；
  - **飞控心跳 → `bin/main` 拉起 `detect_rt` → 连彩色通道 → NPU 推理**的完整链路；
  - 检测进程崩溃后 `bin/main` 的 3 次重启上限、Ctrl-C 信号转发与退出时子进程回收；
  - 相机服务重启、飞控重启时的故障隔离与恢复。
