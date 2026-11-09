# YOLO26n目标检测

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
│   ├── detector.hpp          # 门面：采集+推理流水线、poll、stats
│   └── target_bridge.hpp     # FrameResult → flight::GuidanceTargetSample
├── src/
│   ├── camera_source.cpp
│   ├── yolo_npu.cpp
│   ├── yolo_cpu.cpp
│   ├── yolo_postprocess.{hpp,cpp}
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
| OrbbecSDK 1.10.37 | 仓库内 `src/driver/orbbec` |
| 全志 VIPLite（NPU 后端） | 系统 `/usr/include/vip_lite.h`、`/lib/libNBGlinker.so`、`/lib/libVIPhal.so` |
| ONNX Runtime 1.23.2（CPU 后端） | 仓库内 `third_party/onnxruntime`（aarch64 库；其他架构用系统安装） |
| NPU 模型 | `assets/models/yolo26n.nb`（输入 640×640×3 uint8，输出 8400×4 + 8400×2） |
| CPU 模型 | `assets/models/yolo26n.onnx`（输入 640×640×3 FP32） |

构建开关 `PURE_DETECTION_BUILD`：

- `AUTO`（默认）：依赖齐全才构建，缺依赖时打印 `STATUS` 并跳过，不影响其余 target；
- `ON`：缺依赖直接 `FATAL_ERROR` 并列出缺项；
- `OFF`：不构建。

后端开关 `PURE_DETECTION_BACKEND`：

- `AUTO`（默认）：优先 NPU（VIPLite）；**没有检测到 NPU 时自动改用 CPU（ONNX Runtime）**；
- `NPU`：强制 NPU，缺 VIPLite 则按 `PURE_DETECTION_BUILD` 跳过/FATAL；
- `CPU`：强制 CPU（ONNX Runtime），用于无 NPU 机器或对照测试。

## 3. 构建

```bash
./scripts/autoBuild.sh          # 增量 Release
./scripts/autoBuild.sh -c       # 清理后全量 Release
./scripts/autoBuild.sh -d       # Debug
```

手动构建：

```bash
# 默认 AUTO：有 NPU 用 NPU；没有 NPU 自动用 CPU
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
env -u LD_LIBRARY_PATH -u LD_PRELOAD cmake --build build -j8

# 在本机强制 CPU 对照（建议单独 checkout，或切换前清掉共享产物）
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
  cmake -S . -B build_cpu -DCMAKE_BUILD_TYPE=Release -DPURE_DETECTION_BACKEND=CPU
```

产物：

```text
lib/libdetection_rt.a
bin/detect_rt
```

## 4. 运行

```bash
# 实时相机，处理 100 帧后退出（--frames 0 表示一直运行到 Ctrl-C）
./bin/detect_rt --frames 100

# 实时 + 每帧一行 JSON + 保存带框图
./bin/detect_rt --frames 100 --json --dump-dir /tmp/detect

# 离线单图
./bin/detect_rt --image /path/to/image.jpg --json

# 指定相机参数/阈值
./bin/detect_rt --camera 640x480@60 --conf 0.25 --iou 0.45
```

参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--model PATH` | NPU: `yolo26n.nb`；CPU: `yolo26n.onnx` | 模型路径；默认从当前目录/可执行文件向上查找 |
| `--conf` | `0.25` | 置信度阈值 |
| `--iou` | `0.45` | 逐类 NMS IoU |
| `--image FILE` | — | 离线单图模式 |
| `--frames N` | `0` | 实时模式处理 N 帧后退出，`0` = 无限 |
| `--json` | 关 | 每个处理帧向 stdout 输出一行 JSON，诊断日志走 stderr |
| `--dump-dir DIR` | 关 | 保存带框 JPG |
| `--stream-dir DIR` | 关 | 把带框图原子写成 `DIR/frame.jpg`，供 mjpg-streamer 远程推流 |
| `--stream-fps FPS` | `20` | 推流 JPG 最大写入帧率（1~60） |
| `--camera WxH@FPS` | `640x480@60` | 彩色流 profile |
| `--verbose` | 关 | 放开 OrbbecSDK 日志 |

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

相机只能被一个进程采集：`detect_rt` 用 Orbbec SDK 打开相机时，原来的
`input_uvc` 版 mjpg-streamer 必须停掉。推荐用仓库脚本一键启动：

```bash
./scripts/start_detection_stream.sh --replace
# 浏览器打开 http://192.168.3.163:8080/ （板子 IP 以实际为准）
```

- `--replace` 会先执行 `sudo fuser -k 8080/tcp`，停掉占用端口的旧 mjpg-streamer；
  若已手动停掉可省略。
- 其余参数会原样传给 `detect_rt`，例如：

  ```bash
  ./scripts/start_detection_stream.sh --replace --conf 0.3 --camera 640x480@60
  ```

- 按 `Ctrl-C` 会同时停止 `detect_rt` 与脚本启动的 mjpg-streamer。

脚本内部做两件事：

1. 先启动 `bin/detect_rt --stream-dir /dev/shm/pure-uav-detect --stream-fps 20`，等它写出第一帧；
2. 再启动 mjpg-streamer`input_file.so -f /dev/shm/pure-uav-detect/ -e -d 0.05`
   - `output_http.so -p 8080`。

`detect_rt` 先把带框图写成 `frame_tmp.bin`，再原子 `rename` 成 `frame.jpg`；
临时文件故意不用 `.jpg` 扩展名，`input_file --existing` 扫描目录时不会把它当成视频帧。

不要用 `input_file` 的 inotify 模式（`-r -n frame.jpg`）：写入速率一高，
inotify 事件会堆积，而该插件每次 `read` 只处理第一个事件，实测输出会掉到约 1 FPS。
`--existing` 模式按 `-d` 固定延迟轮询 `frame.jpg`，实测稳定在约 20 FPS。

手工等价命令：

```bash
mkdir -p /dev/shm/pure-uav-detect
MJPG=/home/orangepi/Documents/mjpg-streamer/mjpg-streamer-experimental

# 终端 1：先启动检测，等 /dev/shm/pure-uav-detect/frame.jpg 出现
./bin/detect_rt --stream-dir /dev/shm/pure-uav-detect --stream-fps 20

# 终端 2：再启动 mjpg-streamer
$MJPG/mjpg_streamer \
  -i "$MJPG/input_file.so -f /dev/shm/pure-uav-detect/ -e -d 0.05" \
  -o "$MJPG/output_http.so -p 8080 -w $MJPG/www"
```

停止当前推流：

```bash
pkill -x detect_rt
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
| `infer_ms` | 当前推理后端的前向耗时 |

类别：`0=armor_blue`、`1=armor_red`。

`to_target_sample()` 映射：

| GuidanceTargetSample | 来源 |
| --- | --- |
| `boot_us` | 调用方传入（由 `host_us` 经 SystemTime 换算） |
| `u_px/v_px/w_px/h_px/conf/class_id` | 主目标 |
| `present/has_box` | `present` |
| `has_range/range_m` | 恒 `false/0`（本模块不测距） |
| `has_v_T/v_T` | 恒 `false/零向量`（真机拿不到目标速度） |

**时间戳语义**：`host_us` 是帧到达时刻，不是曝光中点，也不是飞控 boot 时钟；曝光→到达的链路时延本次不补偿。换算成 `GuidanceTargetSample.boot_us` 后可直接供制导使用。

## 7. 范围与限制

- 单进程单实例；不要在多个线程里并发调用同一个 `Detector::poll()` 之外的推理接口。

## 机载共享相机

`bin/main` 同时运行 VIO 和检测，由 `OrbbecCamera` 统一采集彩色/灰度帧。
`DetectorConfig.external_camera=true` 时 `Detector::start()` 只启动推理线程，
宿主用 `submitFrame(bgr, host_us, device_us, seq, host_monotonic_ns)` 投递最新帧。
像素数据通过 `cv::Mat` 引用计数持有，投递后不得修改；停止前必须先停相机投递。
独立 `detect_rt` 仍使用检测模块自己的相机采集器。
