# 同进程目标检测接入

> **架构更新（2026-11-09）**：本树已改为「camera_server 独占采集 + 共享内存双通道 +
> 多进程消费者」。本文描述的「单进程共享相机 + `main` 组合根」是外部工作副本的设计，
> **不是本树的运行方式**。当前实现与运行方法见 [camera-ipc.md](camera-ipc.md)。
> 下文保留作检测侧契约参考（限频、帧龄、会话失效、`TargetSnapshot` 语义）。
> 本树检测算法参数的唯一真源是 `config/detector-rt.yaml`（不是本文的 `app-detection.yaml`），
> 加载接口见 `src/detection-rt/include/detector_config.hpp`；
> 本树检测子进程由 `bin/main` 等飞控心跳后拉起并监督（最多自动重启 3 次），
> 相机服务由外部托管启动，VIO 在 `main` 内运行；`main` 不解析检测算法配置。
> `./bin/main --check-config` 校验应用/VIO 配置；检测配置用 `./bin/detect_rt --check-config` 校验。
> 本文其余单进程检测设计作为历史记录保留。

日期：2026-11-05。只在 `/home/kxy/桌面/pure-uav-test/pure-uav-main` 中修改。

## 实现范围

主程序现在支持：同一台 Orbbec 相机 → VIO + 独立 YOLO 检测线程。
检测产生原图像素坐标框、类别、置信度及置信度最高的主目标中心和宽高。
没有实现目标跟踪 ID、测距、目标世界坐标、目标跟随或制导，也不会发送飞行控制命令。

```text
唯一的相机采集线程
  ├─ 原有灰度/深度 → 有界 VIO 队列 → VIO 线程 → 主循环 → 飞控 ODOMETRY
  └─ 同一彩色帧的编码数据 → 最新帧槽 → 检测线程解码/NPU/后处理 → 目标快照
```

- 保留 VIO 已有的灰度解码和时标；不会为检测修改分辨率、曝光策略或标定。
- 开启检测时才分发彩色数据。不使用灰度伪造彩色，不额外创建 Pipeline。
- 在采集回调中限频，默认最多 10 Hz；只复制被选中的编码数据。通常 MJPG 无须先复制整张 BGR。
- 数据复制在检测互斥锁之外，解码、推理、NMS 全在检测线程；主循环不等待推理完成。
- 只保留一张待推理图像，新的替换旧的；输入帧所有权独立于 SDK 缓冲生命周期。
- 无目标是有效的新结果，会覆盖旧目标；超龄、异常、停止和会话/时间基变化使旧目标失效。
- 会话变化会同时废弃待处理图像与已经在推理的旧结果，不把旧框标成新会话目标。
- 检测故障记录错误、清空目标并停用检测，不设置主程序停止标志，VIO 可以继续运行。

`TargetSnapshot` 使用和 VIO 相同的主机单调采集时间 `captured_host_ns`。
原 `FrameResult.host_us` 保留 Unix 微秒语义，新路径不提供它，填 0；不得把单调时钟冒充 Unix 或飞控时间。
暂不提供推算的飞控 boot 时间。将来做制导必须另行做采样时间对齐，不得直接使用打印时刻。

## 配置与启用

- `config/app.yaml`：仍默认关闭检测，保留本周 VIO-only 验收路径。
- `config/app-detection.yaml`：完整的 VIO + 检测配置，检测开关为 true。
- `model_path` 相对本副本的仓库根目录解析，默认原有 `assets/models/yolo26n.nb`。
- `num_classes=2` 是现有模块的模型约定，不代表这里重新验证了权重语义。
  现有类别约定为 0=蓝装甲、1=红装甲，上板后必须用实物核对。`target_class=-1` 接受所有类别，0/1 可限定类别。
- `max_hz=10` 控制检测负载；`max_age_ms=200` 同时限制采集年龄和输出年龄，过期结果不会保持 present。
- 模型必须是现有 NPU 导出的 640 输入、8400 个已解码 YOLO xywh + 类别概率布局，不支持随意替换的分类、raw-head 或 end-to-end 输出。

在实际 NPU 板上从源代码重新构建，不要拷贝开发机的 x86 二进制和 CMakeCache：

```bash
cmake -S . -B build-uav -DCMAKE_BUILD_TYPE=Release -DPURE_DETECTION_BUILD=ON
cmake --build build-uav --target main -j2
./bin/main --config config/app-detection.yaml --check-config
```

上述检查不创建相机、不加载模型、不打开串口。
只有设备和台架准备好后才手动执行实际运行：

```bash
./bin/main --config config/app-detection.yaml --no-send
```

`--no-send` 仍接收飞控 IMU、采集图像并运行估计，但禁止发送 ODOMETRY；它不是“无硬件模拟”模式。
台架确认后去掉该参数才允许发送定位。检测框始终只是本地结果，不会发送成飞控导航目标。

状态行 `[DETECTION]` 包含 running、valid、present、class、conf、center、size、boxes、处理数、
最新槽替换数、限频数、过期数、推理耗时、采集年龄和最后错误。打印周期仍由 status_period_s 决定，
不是每帧打印；进程内调用 latest() 可以获取最新结果及全部框。

## 依赖与测试边界

真实 `.nb` 推理仍依赖板端 `vip_lite.h`、`libNBGlinker.so`、`libVIPhal.so`。
开发机没有这些依赖：构建可验证采集接口、工作线程、解码和后处理；不能执行真实 NPU 推理。
在缺少 NPU 的构建中打开检测配置会在配置校验阶段返回 2，不会静默假装启用了检测。

新增测试使用明确标识的测试后端验证线程行为，使用已知张量验证后处理。它们不证明模型精度或板端性能。
原生 NPU 的输入映射、缓存操作或执行失败现在通过返回值上报，不能沿用上次输出伪造成功。
两个后端 checked 函数保留旧 awnn API 的包装入口；这部分仍需板端编译、链接和真实 NPU 验证。

已知限制：单进程线程无法隔离供应商 NPU 驱动的原生崩溃；如果驱动调用永久挂起，
检测不会向主循环提供新结果，但停止时等待推理线程可能仍会受阻。需要更强故障隔离时应另行评估进程隔离。
本轮不通过强制杀线程、伪输出或隐藏推理错误来掩盖该风险。

## 上机验收

先关闭检测完成 VIO→飞控验收，再开启检测比较相同运动轨迹和光照下的：

1. 主循环 overrun、IMU 频率与丢包、VIO 健康/重置数。
2. 实际定位接收率、VIO 采集年龄和发送失败数，不能只看检测 FPS。
3. 实物蓝/红类别、置信度、框中心、目标消失后 present 清除。
4. 遮挡、飞控重启和断线恢复后是否还能看到旧目标。
5. CPU、内存、温度及供电。若检测影响定位，先关闭检测或降低 max_hz，不放宽定位失效门限。

## 本轮软件验证

以下是检测接入阶段的历史记录。后续全量构建、输入量化、legacy 桥接和通信新鲜度修复，
以 acceptance-fixes.md 和 build-fix/verification-fixes.json 为准。

- x86_64：main、vio_live 以及新增检测测试均构建成功，项目代码无编译告警。
- 选定的 16 项无硬件回归全部通过；检测线程、YOLO 后处理与 VIO 线程各重复运行 10 轮通过。
- 从 /tmp 检查默认配置和 --no-send 配置均返回 0；启用 NPU 检测的配置在本机明确返回 2，说明缺少真实 NPU 依赖。
- 启用分支的 NPU 适配器和旧目标桥接通过语法检查，不代表原生 NPU 实现已编译、链接或实际推理。
- 没有打开真实相机或飞控串口；没有测量模型精度、板端耗时或飞控融合。
- 原工程 1015 项文件/符号链接与复制前基线一致；副本的 28 项模型、标定和既有占位配置未变。

记录：build/verification-detection.json、build/build-detection-final.log、
build/ctest-detection-final.log、build/ctest-detection-repeat.log。

## 后续修复边界

UINT8 未量化输入现在使用恒等 LUT；INT8 未量化输入明确报错，FP32 不使用 LUT。
TF_ASYMM 与 DFP 仍按模型给出的量化参数计算；纯函数单测不需要 NPU。
NPU 参数数组、句柄数组及部分创建失败的回收归到同一条清理路径。
真实 VIPLite 编译/链接、失败注入和重复加载泄漏测试仍须在板端执行。

旧 Detector/CameraSource 仍是单独的 legacy 台架入口，不能与 main 同时占用相机。
其 alive() 现在同时要求推理运行标志和 camera.running()；采集异常退出会清除运行标志。
legacy target_bridge 对非有限像素、非正尺寸、非法置信度/类别拒绝并清空输出。

DetectionRuntime::start() 的 ready 等待仍无可安全强制中断的初始化期限。
不能用 wait_for 后的无界 join 宣称解决；这项按当前线程和供应商 API 约束保留为明确遗留。
