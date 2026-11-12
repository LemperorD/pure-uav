# factory_core

本目录提供检测、VIO、制导的抽象工厂及现有模块适配层。创建工厂/产品只构造对象；调用 `init()` 才启动运行时。相机、串口和共享内存连接仍由宿主配置和管理。

## 接口与生命周期

- `IProduct` 提供虚析构、`init()`、`stop()`、`initialized()`。初始化失败抛出 `std::runtime_error`，缺失依赖抛出 `std::invalid_argument`。
- `init()` 和 `stop()` 均可重复调用，停止后可重新初始化；析构停止并等待产品内部工作线程退出。`initialized()` 表示初始化状态，运行时健康情况须读取结果及发布门控。
- 所有工厂返回 `std::shared_ptr`；每次创建产生独立产品和运行时。VIO、制导产品保留数据源的共享所有权，工厂销毁后产品仍可使用。
- 宿主串行调用产品生命周期和单步接口；不要在 `publication()` 锁存活期间调用 `stop()` 或释放产品。发布锁也不能比产品存活更久。
- 一个破坏性读取队列只能交给一个运行中的消费者；共享指针保障存活时间，不负责广播或消费仲裁。

| 产品 | 原模块 | 调用约定 |
| --- | --- | --- |
| `YoloA733Detector` | `DetectionRuntime` + `YoloBackend` + 原后处理 | `submit(frame)` 投递彩色帧，`detect(out, now_ns)` 读取最新有效快照；`setEpoch` 传播会话/代数切换并清除旧结果 |
| `OpenVINS` | `VioRuntime` | 注入 IMU/图像数据源，`update(out)` 读取最新定位；实际发布使用 `publication()` 与原宿主门控 |
| `PNGuidance` | `GuidanceLoop` | 注入惯导、目标、指令出口，`guide(out)` 执行一拍并原样返回状态码；宿主负责按配置频率调度 |

检测使用 CMake 选择的 NPU 或 CPU YOLO 后端；开发机需设置有效 ONNX 模型路径，板端使用对应的 NB 模型。也可通过 `DetectionProductConfig::create_backend` 注入后端，每次调用必须返回独立实例。适配层不打开相机，彩色帧由宿主从现有相机服务取得，时间戳使用 `steady_clock` 纳秒。

`detect` / `update` 是快照读取，同一快照可能被重复读取。无有效结果时返回 `false`，调用方不得使用其中的旧结果。VIO 快照的 `valid` 不代表可以向飞控发布，仍需使用原有健康、会话、时基及新鲜度检查。

## 使用

```cpp
#include "pure_uav_factory.hpp"

// imu、frames、ins、target、sink 是宿主已配置的数据端口 shared_ptr。
DetectionProductConfig detection;
detection.runtime.model_path = model_path;

VioProductConfig vio;
vio.options.config_dir = calibration_dir;
vio.imu = imu;
vio.frames = frames;

GuidanceProductConfig guidance;
guidance.params = guidance_params;
guidance.loop = loop_config;  // 必须包含真实相机内参与外参。
guidance.ins = ins;
guidance.target = target;
guidance.sink = sink;

PureUavFactory factory(detection, vio, guidance);
auto detector = factory.createDetectorFactory()->createDetector();
auto localization = factory.createVIOFactory()->createVIO();
auto controller = factory.createGuidanceFactory()->createGuidance();

// 顺序启动；若后续初始化失败，RAII 会清理此前成功启动的产品。
detector->init();
localization->init();
controller->init();
// 宿主循环中投递图像、读取结果、执行制导，并处理返回状态。
// 退出时先停止制导，再停止定位和检测；端口在产品停止后关闭。
controller->stop();
localization->stop();
detector->stop();
```

## 构建与测试

目录内 `CMakeLists.txt` 定义 `factory_core` 库，依赖已有 `detection_rt`、`pure_vio_runtime`、`guidance_flight` 目标。在这些目标注册后可通过 `add_subdirectory(src/factory_core)` 接入宿主，再链接 `factory_core`。

本次按目录修改范围保留原有根构建文件与 `src/app/factory_main.cpp`。因此根工程不会自动构建本模块；旧草稿入口的头文件路径、抽象类实例化、裸指针用法和线程退出仍需在后续接入时更新。现有应用入口不受影响。

启用 `PURE_FACTORY_BUILD_TESTS=ON` 后生成 `test_factory_core`，可执行 `ctest -R '^factory_core$' --output-on-failure`。测试用内存数据源和伪检测后端，不连接相机或飞控；覆盖检测结果与会话切换、初始化失败恢复、幂等初始化/停止、VIO 实际配置加载与重启、制导发送错误、依赖存活时间和析构清理。测试会读取仓库内 `gemini_1920x1080` VIO 配置，不验证板端 NPU 推理或真实飞行链路。
