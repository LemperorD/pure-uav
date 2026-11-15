# YOLO26n目标检测

> **现行运行方式**：相机由 `camera_server` 独占采集，`detect_rt` 从共享内存彩色通道取帧，
> 检测走 `DetectionRuntime`。检测算法参数的**唯一真源**是
> [`config/detector-rt.yaml`](../../config/detector-rt.yaml)，不提供对应命令行选项。
> 下文凡标注「旧台架」的内容只描述遗留 `Detector` / `CameraSource` 实现。

```text
Orbbec 彩色流 640×480@60
        │  MJPG → BGR
        ▼
letterbox 640×640
        ▼
VIPLite / awnn  →  assets/models/yolo26n.nb
        ▼
输出融合 [4,8400] + [2,8400] → [6,8400]
        ▼
解码 + 逐类 NMS → FrameResult
        ▼
poll(FrameResult&) → to_target_sample() → flight::GuidanceTargetSample
```

## 1. 目录结构

```text
src/detection-rt/
├── include/
│   ├── detection_types.hpp   # FrameResult / DetectionBox（固定容量，实时无堆分配）
│   ├── camera_source.hpp     # Orbbec 彩色采集接口
│   ├── yolo_output.hpp       # NPU/CPU 共用的输出视图
│   ├── yolo_npu.hpp          # NPU 后端：NBG 加载 / 量化 / 输出融合
│   ├── yolo_cpu.hpp          # CPU 后端：ONNX Runtime
│   ├── yolo_backend.hpp      # 编译期后端别名 YoloBackend
│   ├── detection_runtime.hpp # RuntimeConfig / DetectionRuntime
│   ├── detector_config.hpp   # config/detector-rt.yaml 加载接口
│   ├── detector.hpp          # 旧台架门面：采集+推理流水线、poll、stats
│   └── target_bridge.hpp     # FrameResult → flight::GuidanceTargetSample
├── src/
│   ├── camera_source.cpp
│   ├── yolo_npu.cpp
│   ├── yolo_cpu.cpp
│   ├── yolo_postprocess.{hpp,cpp}
│   ├── detection_runtime.cpp
│   ├── detector_config.cpp
│   ├── detector.cpp
│   ├── target_bridge.cpp
│   └── detect_rt_main.cpp    # CLI
├── third_party/awnn/         # 全志 ai-sdk awnn 运行时（NPU）
├── third_party/onnxruntime/  # ONNX Runtime 1.23.2（CPU）
└── CMakeLists.txt
```

## 2. 依赖

| 依赖 | 说明 |
| --- | --- |
| OpenCV 4.5.4 | 图像解码、letterbox、NMS |
| OrbbecSDK 1.10.37 | 仓库内 `src/driver/orbbec`（仅遗留台架与相机服务使用） |
| yaml-cpp | 解析 `config/detector-rt.yaml` |
| 全志 VIPLite（NPU 后端） | 系统 `/usr/include/vip_lite.h`、`/lib/libNBGlinker.so`、`/lib/libVIPhal.so` |
| ONNX Runtime 1.23.2（CPU 后端） | 仓库内 `third_party/onnxruntime`（aarch64 库；其他架构用系统安装） |
| NPU 模型 | `assets/models/yolo26n.nb`（输入 640×640×3 uint8，输出 8400×4 + 8400×2） |
| CPU 模型 | `assets/models/yolo26n.onnx`（输入 640×640×3 FP32） |

后端自动选择，无需配置开关：

- 先探测 `vip_lite.h`、`libNBGlinker.so`、`libVIPhal.so`，齐全即用 NPU；
- 缺任一项则改用 CPU（ONNX Runtime）；
- OpenCV 4 / OrbbecSDK 缺失，或两种后端都不可用时跳过本模块，不影响其它 target。

## 3. 构建

```bash
./scripts/autoBuild.sh          # 增量 Release
./scripts/autoBuild.sh -c       # 清理后全量 Release
./scripts/autoBuild.sh -d       # Debug
```

手动构建：

```bash
# 有 NPU 用 NPU；没有 NPU 自动用 CPU
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
env -u LD_LIBRARY_PATH -u LD_PRELOAD cmake --build build -j8
```

产物：

```text
lib/libdetection_rt.a
bin/detect_rt
```

## 4. 运行

检测算法参数只从 `config/detector-rt.yaml` 读取（可用 `--config FILE` 改路径），
启动时打印解析后的配置摘要。字段缺失、未知字段、旧占位字段、越界值或模型文件不存在
都会直接报错退出，不存在隐藏的 C++ 默认值。`--check-config` 只校验配置，
不加载模型、不连接相机服务。

```bash
# 只校验配置
./bin/detect_rt --check-config

# 实时：先独立启动 camera_server，VIO 由 main 内部运行，检测由 bin/main 心跳后拉起
./bin/main --detect-arg --frames --detect-arg 100 --detect-arg --json \
  --detect-arg --dump-dir --detect-arg /tmp/detect

# 单独调试检测时也可直接运行（不要与 bin/main 同时运行，避免两个消费者）
./bin/detect_rt --frames 100 --json --dump-dir /tmp/detect

# 离线单图（共用同一份配置）
./bin/detect_rt --image /path/to/image.jpg --json

# 指定另一份配置
./bin/detect_rt --config /tmp/detector-alt.yaml --check-config
```

运行参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--config FILE` | `config/detector-rt.yaml` | 配置文件路径 |
| `--check-config` | 关 | 只校验配置后退出 |
| `--image FILE` | — | 离线单图模式 |
| `--frames N` | `0` | 实时模式处理 N 帧后退出，`0` = 无限 |
| `--json` | 关 | 每个处理帧向 stdout 输出一行 JSON，诊断日志走 stderr |
| `--dump-dir DIR` | 关 | 保存带框 JPG |
| `--stream-dir DIR` | 关 | 把带框图原子写成 `DIR/frame.jpg`，供 mjpg-streamer 远程推流 |
| `--stream-fps FPS` | `20` | 推流 JPG 最大写入帧率（1~60） |
| `--shm NAME` | `/pure_uav_camera_color` | 彩色共享内存通道 |
| `--shm-timeout-ms N` | `5000` | 等待通道就绪的超时 |
| `--verbose` | 关 | 兼容旧脚本，无实际作用 |

检测算法参数（唯一真源 `config/detector-rt.yaml`，顶层键 `detection-rt:`，全部必填）：

| 字段 | 当前值 | 说明 |
| --- | --- | --- |
| `model_path` | `auto` | `auto` 按编译后端选 `yolo26n.onnx` / `.nb`，也可写显式路径 |
| `imgsz` | `640` | 当前后端只支持 640 |
| `num_classes` | `2` | 模型类别数（1~1000） |
| `conf_thr` | `0.25` | 置信度阈值 (0,1] |
| `iou_thr` | `0.45` | 逐类 NMS IoU (0,1] |
| `max_det` | `100` | 单帧最大框数（1~300） |
| `target_class` | `-1` | `-1` 接受所有类别；否则只保留指定类别 |
| `max_hz` | `10` | 检测侧限频（1~60），与 `config/camera.yaml` 的服务端限频是两道独立门限 |
| `max_age_ms` | `200` | 输入帧与结果的最大帧龄（1~2000） |

若终端本身注入了 IDE 的旧版 libstdc++，运行前加：

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD ./bin/detect_rt --frames 100
```

### 4.1 远程实时查看带框画面

依赖`mjpg-streamer`，需要先在`../`中执行`git clone https://github.com/jacksonliam/mjpg-streamer.git`并编译安装。

> `mjpg-streamer`的使用方式：
>
> ```bash
> # ubuntu系统
> sudo apt-get install -y cmake libjpeg8-dev
> # debian系统
> sudo apt-get install -y cmake libjpeg62-turbo-dev
> git clone https://github.com/jacksonliam/mjpg-streamer.git
> cd mjpg-streamer/mjpg-streamer-experimental
> make -j4
> sudo make install
> export LD_LIBRARY_PATH=.
> # video0按实际设备序号修改(使用v4l2-ctl --list-devices查看)
> sudo ./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -u -f 30" -o "./output_http.so -w ./www"
> ```
>
> 在浏览器中输入 `http://192.168.3.163:8080/` 查看实时画面。

相机只由 `camera_server` 采集，本模块不再打开设备。画面推流分两步：

```bash
# 终端 A：先独立启动 camera_server，VIO 由 main 内部运行；main 拉起检测并写 frame.jpg
./bin/main --detect-arg --stream-dir --detect-arg /dev/shm/pure-uav-detect \
  --detect-arg --stream-fps --detect-arg 20
# 或单独调试：./bin/detect_rt --stream-dir /dev/shm/pure-uav-detect --stream-fps 20

# 终端 B：只启动 mjpg-streamer 前端
./scripts/start_detection_stream.sh --replace
# 浏览器打开 http://<板子 IP>:8080/
```

- `--replace` 会先执行 `sudo fuser -k 8080/tcp`，停掉占用端口的旧 mjpg-streamer；
  若已手动停掉可省略。
- 推流脚本只负责 mjpg-streamer，不再启动 `detect_rt`，也不接受检测参数；
  检测算法参数统一改 `config/detector-rt.yaml`。
- 按 `Ctrl-C` 停止 mjpg-streamer；先独立启动 camera_server，VIO 由 main 内部运行，
  检测由 `bin/main` 拉起（或单独运行 `detect_rt`），停止 `bin/main` 即停止检测子进程。

脚本内部流程：等待 `detect_rt` 写出首帧 `frame.jpg`，
再用 `input_file.so -f /dev/shm/pure-uav-detect/ -e -d <延迟>` 读取，
由 `output_http.so -p 8080` 输出。

`detect_rt` 先把带框图写成 `frame_tmp.bin`，再原子 `rename` 成 `frame.jpg`；
临时文件故意不用 `.jpg` 扩展名，`input_file --existing` 扫描目录时不会把它当成视频帧。

不要用 `input_file` 的 inotify 模式（`-r -n frame.jpg`）：写入速率一高，
inotify 事件会堆积，而该插件每次 `read` 只处理第一个事件，实测输出会掉到约 1 FPS。
`--existing` 模式按 `-d` 固定延迟轮询 `frame.jpg`，实测稳定在约 20 FPS。

手工等价命令：

```bash
mkdir -p /dev/shm/pure-uav-detect
MJPG=/home/orangepi/Documents/mjpg-streamer/mjpg-streamer-experimental

# 终端 1：先启动检测（由 main 拉起，或单独运行），等 frame.jpg 出现
./bin/main --detect-arg --stream-dir --detect-arg /dev/shm/pure-uav-detect \
  --detect-arg --stream-fps --detect-arg 20

# 终端 2：再启动 mjpg-streamer
$MJPG/mjpg_streamer \
  -i "$MJPG/input_file.so -f /dev/shm/pure-uav-detect/ -e -d 0.05" \
  -o "$MJPG/output_http.so -p 8080 -w $MJPG/www"
```

停止当前推流：

```bash
pkill -x main        # main 退出时会先停止 detect_rt 子进程
pkill -x detect_rt   # 仅在单独运行 detect_rt 时需要
pkill -f "output_http.so -p 8080"
```

## 5. 库接口

```cpp
#include "detector.hpp"
#include "target_bridge.hpp"

pure_uav::detection::DetectorConfig cfg;
cfg.model_path = "assets/models/yolo26n.nb";   // CPU 版改成 yolo26n.onnx

pure_uav::detection::Detector det;
std::string err;
if (!det.start(cfg, err)) { /* err 含相机/NBG 失败原因 */ }

pure_uav::detection::FrameResult r;
if (det.poll(r)) {                              // 非阻塞；只返回最新结果
    pure_uav::guidance::flight::GuidanceTargetSample t;
    // boot_us：用通信模块 SystemTime 把 r.host_us 换算到飞控 boot 时钟
    pure_uav::detection::to_target_sample(r, boot_us, t);
    // t.present=false ⇒ 主循环可进入 SEARCH
}
det.stop();                                     // 停线程、释放相机和推理后端
```

- `poll()` 无新结果时返回 `false`，且不修改传入的 `out`；
- `stats()` 给出采集 FPS、处理 FPS、平均推理 ms、采集/处理帧数；
- `latest_frame(cv::Mat&)` 返回最近结果对应的 BGR 帧（浅拷贝），仅供可视化；
- 同一进程只允许一个 `Detector` 实例（awnn 是进程级全局状态）；
- 只处理最新帧，旧帧可丢，适合定周期主循环。

## 6. 数据格式与时间戳

`FrameResult` 关键字段：

| 字段 | 含义 |
| --- | --- |
| `host_us` | 帧到达主机的 Unix 微秒（`system_clock`） |
| `device_us` | 相机设备时间戳，仅诊断 |
| `seq` | 相机帧序号，单调递增 |
| `width/height` | 原始图像尺寸 |
| `count/present` | 有效框数量 / 是否有目标 |
| `boxes[]` | `x1,y1,x2,y2, conf, class_id`（原始图像像素坐标） |
| `u_px/v_px/w_px/h_px/conf/class_id` | 置信度最高的主目标，直接对应 `GuidanceTargetSample` |
| `has_range/range_m` | 主目标有效测距 / 彩色相机中心到表面的直线距离（m） |
| `infer_ms` | 当前推理后端的前向耗时 |

类别：`0=armor_blue`、`1=armor_red`。

`to_target_sample()` 映射：

| GuidanceTargetSample | 来源 |
| --- | --- |
| `boot_us` | 调用方传入；共享采集路径由 `captured_host_ns` 经主程序时钟映射换算 |
| `u_px/v_px/w_px/h_px/conf/class_id` | 主目标 |
| `present/has_box` | `present` |
| `has_range/range_m` | 同帧原生深度投影的直线距离（m）；无有效深度/标定时为 `false/0` |
| `has_v_T/v_T` | 恒 `false/零向量`（真机拿不到目标速度） |

**时间戳语义**：旧直采接口的 `host_us` 是主机 Unix 到达时间；共享采集路径将其置 0，以 `TargetSnapshot.captured_host_ns` 保留 CLOCK_MONOTONIC 采集时刻。该值由主程序换算为飞控 boot 时间。测距、标定与 V2 负载约定见 [depth-guidance.md](../../assets/docs/app/depth-guidance.md)。

## 7. 范围与限制

- 单进程单实例；不要在多个线程里并发调用同一个 `Detector::poll()` 之外的推理接口。

## 机载共享相机（2026-11-11：V2 同帧深度测距）

相机只由 `camera_server` 初始化一次；`detect_rt` 是**彩色共享内存通道的只读消费者**，
不再使用 `CameraSource` 打开设备。检测侧走 `DetectionRuntime::submit(EncodedColorFrame)`：
原始 MJPG 字节在检测进程内解码，带 10 Hz 限频、200 ms 帧龄与会话失效保护。

```bash
./bin/detect_rt --check-config                  # 只校验检测配置
# 先启动 camera_server，再由 main 同时运行 VIO 与检测进程监督；勿同时运行 vio_live
./bin/main --detect-arg --shm --detect-arg /pure_uav_camera_color \
  --detect-arg --shm-timeout-ms --detect-arg 10000 --detect-arg --json
# 单独调试检测（不要与 bin/main 同时运行）
./bin/detect_rt --shm /pure_uav_camera_color --shm-timeout-ms 10000 --json
./bin/detect_rt --image test.jpg                # 离线单图，不连接相机服务
```

`--camera WxH@FPS` 已移除（分辨率/曝光由 `config/camera.yaml` 统一配置）。
`--model` / `--conf` / `--iou` 等检测算法参数也已移除，唯一真源为
`config/detector-rt.yaml`；`--frames/--json/--dump-dir/--stream-dir/--stream-fps`
等运行参数保持原语义。完整协议与互斥清单见 `assets/docs/app/camera-ipc.md`。

### 遗留接口

`Detector` / `CameraSource` 源码保留给既有测试，不再是实时运行路径。
`DetectorConfig.external_camera=true` 时 `Detector::start()` 只启动推理线程、不打开设备，
宿主用 `submitFrame(bgr, host_us, device_us, seq, host_monotonic_ns)` 投递不可变 BGR 帧；
停止前必须先停帧投递再调用 `Detector::stop()`。
任何直采工具（`test_camera`、`depth_range_probe`、V4L2 标定脚本）都禁止与 `camera_server` 同时运行。
