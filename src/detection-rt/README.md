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
poll(FrameResult&) → to_target_sample() → app::TargetSample
```

## 1. 目录结构

```text
src/detection-rt/
├── include/pure_uav/detection/
│   ├── detection_types.hpp   # FrameResult / DetectionBox（固定容量，实时无堆分配）
│   ├── camera_source.hpp     # Orbbec 彩色采集接口
│   ├── yolo_npu.hpp          # NBG 加载 / 预处理 / 推理 / 输出融合
│   ├── detector.hpp          # 门面：采集+推理流水线、poll、stats
│   └── target_bridge.hpp     # FrameResult → app::TargetSample
├── src/
│   ├── camera_source.cpp
│   ├── yolo_npu.cpp
│   ├── yolo_postprocess.{hpp,cpp}
│   ├── detector.cpp
│   ├── target_bridge.cpp
│   └── detect_rt_main.cpp    # CLI
├── third_party/awnn/         # 全志 ai-sdk awnn 运行时
└── CMakeLists.txt
```

## 2. 依赖

| 依赖 | 说明 |
| --- | --- |
| OpenCV 4.5.4 | 图像解码、letterbox、NMS |
| OrbbecSDK 1.10.37 | 仓库内 `src/driver/orbbec` |
| 全志 VIPLite | 系统 `/usr/include/vip_lite.h`、`/lib/libNBGlinker.so`、`/lib/libVIPhal.so` |
| 模型 | `assets/models/yolo26n.nb`（输入 640×640×3 uint8，输出 8400×4 + 8400×2） |

构建开关 `PURE_DETECTION_BUILD`：

- `AUTO`（默认）：依赖齐全才构建，缺依赖时打印 `STATUS` 并跳过，不影响其余 target；
- `ON`：缺依赖直接 `FATAL_ERROR` 并列出缺项；
- `OFF`：不构建。

## 3. 构建

```bash
./scripts/autoBuild.sh          # 增量 Release
./scripts/autoBuild.sh -c       # 清理后全量 Release
./scripts/autoBuild.sh -d       # Debug
```

手动构建：

```bash
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
| `--model PATH` | `assets/models/yolo26n.nb` | NBG 路径；默认从当前目录/可执行文件向上查找 |
| `--conf` | `0.25` | 置信度阈值 |
| `--iou` | `0.45` | 逐类 NMS IoU |
| `--image FILE` | — | 离线单图模式 |
| `--frames N` | `0` | 实时模式处理 N 帧后退出，`0` = 无限 |
| `--json` | 关 | 每个处理帧向 stdout 输出一行 JSON，诊断日志走 stderr |
| `--dump-dir DIR` | 关 | 保存带框 JPG |
| `--camera WxH@FPS` | `640x480@60` | 彩色流 profile |
| `--verbose` | 关 | 放开 OrbbecSDK 日志 |

若终端本身注入了 IDE 的旧版 libstdc++，运行前加：

```bash
env -u LD_LIBRARY_PATH -u LD_PRELOAD ./bin/detect_rt --frames 100
```

## 5. 库接口

```cpp
#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/target_bridge.hpp"

pure_uav::detection::DetectorConfig cfg;
cfg.model_path = "assets/models/yolo26n.nb";   // 其余用默认值

pure_uav::detection::Detector det;
std::string err;
if (!det.start(cfg, err)) { /* err 含相机/NBG 失败原因 */ }

pure_uav::detection::FrameResult r;
if (det.poll(r)) {                              // 非阻塞；只返回最新结果
    pure_uav::app::TargetSample t;
    // boot_us：用通信模块 SystemTime 把 r.host_us 换算到飞控 boot 时钟
    pure_uav::detection::to_target_sample(r, boot_us, t);
    // t.present=false ⇒ 主循环可进入 SEARCH
}
det.stop();                                     // 停线程、释放相机和 awnn 上下文
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
| `u_px/v_px/w_px/h_px/conf/class_id` | 置信度最高的主目标，直接对应 `TargetSample` |
| `infer_ms` | NPU 前向 + 输出融合耗时 |

类别：`0=armor_blue`、`1=armor_red`。

`to_target_sample()` 映射：

| TargetSample | 来源 |
| --- | --- |
| `boot_us` | 调用方传入（由 `host_us` 经 SystemTime 换算） |
| `u_px/v_px/w_px/h_px/conf/class_id` | 主目标 |
| `present/has_box` | `present` |
| `has_range/range_m` | 恒 `false/0`（本模块不测距） |
| `has_v_T/v_T` | 恒 `false/零向量`（真机拿不到目标速度） |

**时间戳语义**：`host_us` 是帧到达时刻，不是曝光中点，也不是飞控 boot 时钟；曝光→到达的链路时延本次不补偿。换算成 `TargetSample.boot_us` 后可直接供制导使用。

## 7. 范围与限制

- 单进程单实例；不要在多个线程里并发调用同一个 `Detector::poll()` 之外的 NPU 接口。
