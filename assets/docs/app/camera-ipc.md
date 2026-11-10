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

统一入口 `scripts/start_uav.sh` 按「清理通道 → camera_server → vio_live / detect_rt」顺序拉起，
退出时逆序停止。`scripts/start_detection_stream.sh` 现在只负责 mjpg-streamer 前端。

进程隔离：检测进程原生 NPU 崩溃或挂起不会拖垮 VIO。`start_uav.sh` 检测到 `detect_rt` 退出时
最多自动重启 3 次，期间 `vio_live` 与 `camera_server` 保持运行；只有相机服务或 VIO 退出才整体停止。

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
- 共享内存权限 `0660`；`shm_unlink` 在 `ShmChannelWriter::close()` 与统一入口清理时都会执行。
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
- 心跳 `heartbeat_ns` 由写者每帧更新。`vio_live` 2 s 无心跳退出、`detect_rt` 3 s 无心跳退出，
  由统一入口重新拉起；慢消费者只丢自己的旧帧，不影响采集。
- 彩色通道在服务端按 `color_max_hz`（默认 10）限频；`DetectionRuntime` 的
  `max_hz` / `max_age_ms` / `seq` 单调校验保留为二道防线。

## 5. 构建与运行

```bash
cmake -S . -B build-uav -DCMAKE_BUILD_TYPE=Release
cmake --build build-uav -j2

./bin/camera_server --check-config                 # 不打开相机，只校验 config/camera.yaml
./scripts/start_uav.sh --no-send                   # 只跑 VIO + 相机服务
./scripts/start_uav.sh --detection --no-send       # VIO + 检测
./scripts/start_uav.sh --detection --stream        # 额外写带框 frame.jpg
./scripts/start_detection_stream.sh --replace      # 另一个终端：mjpg-streamer 前端
```

`config/camera.yaml` 是相机参数唯一真源（分辨率/帧率/深度/曝光/测光/通道名/槽大小/彩色限频）。
`vio_live` 的 `--width/--height/--fps` 与曝光类 CLI 只用于兼容旧脚本，不再控制硬件，
启动时若与通道元数据不一致会打印警告；VIO 标定始终以 camera_server 配置为准。

`detect_rt --image FILE` 仍是离线单图模式，不连接相机服务。

## 6. 互斥清单（禁止与 camera_server 同时运行）

| 入口 | 处理 |
|---|---|
| `bin/main` | 已移除 `Detector::start()`，只是通信示例，打印统一入口提示后退出 |
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
  - `camera_server --check-config`、两个消费者 `--help`、无相机服务时消费者明确拒绝启动；
  - 全量 `ctest`：22 项中 20 通过。`onboard_runtime` 随 `external_camera` 修复转为通过；
    `guidance` / `state_adapter` 是制导模块既有失败，本次未改动 `src/guidance/**`，与相机改造无关。
- 必须在板端/真机验证，**不得用本机结果代替**：
  - 真实 Orbbec 采集 + 深度对齐、30 fps 下共享内存写带宽与 `/dev/shm` 容量；
  - 真实 `.nb` NPU 推理（`vip_lite.h` / `libNBGlinker.so` / `libVIPhal.so`）与类别语义；
  - VIO 时间同步、IMU 融合、飞控 ODOMETRY 接收率与检测开启前后的定位质量对比；
  - 检测进程崩溃、相机服务重启、飞控重启时的故障隔离与恢复。
