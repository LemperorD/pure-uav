# 应用抽象工厂

工厂结构沿用参考工程，子工厂只保存配置并创建产品：

```text
IPureUavFactory（PureUavFactory）
├─ IDetectFactory（YoloProcessDetectFactory）→ IDetector（YoloProcessDetector）
├─ IVIOFactory（OpenVINSFactory）→ IVIO（OpenVINS）
└─ IGuidanceFactory（PNGuidanceFactory）→ IGuidance（PNGuidance）
```

`abstract_factory.hpp` 定义接口，`pure_uav_factory.hpp` 实现工厂，
`product.hpp` / `product.cpp` 适配现有运行模块。总工厂只接收检测、VIO、制导
三份配置；三个子工厂的 `create` 方法均为无参调用。

## 入口

`src/app/main.cpp` 解析参数后执行以下步骤：

```cpp
UavApplication app(options);
auto factory = app.prepare();
if (!factory) {
    return app.result();
}

auto vio = factory->createVIOFactory()->createVIO();
auto guidance = options.mission ? factory->createGuidanceFactory()->createGuidance() : nullptr;
auto detector = options.no_detection ? nullptr : factory->createDetectorFactory()->createDetector();

app.init(vio, guidance, detector);
return app.run();
```

`UavApplication` 位于 `src/app/runtime/uav_application.{hpp,cpp}`，负责宿主运行：

- `prepare()` 加载与检查配置、连接飞控和相机、核对测距与标定，然后组合三份产品配置，
  返回抽象总工厂。检查配置后退出、准备失败或收到退出信号时返回空，退出码由 `result()` 提供。
- `init()` 通过 `setVIO` / `setGuidance` 连接产品依赖，再依次初始化 VIO、制导和检测。
  主函数不负责目标通道 fd 或运行时依赖接线。
- `run()` 按配置节拍发布 VIO、调度任务、监督相机和检测、处理返航及降落兜底。
- 宿主退出时先停止任务发送，再停止检测、VIO、相机和通信；异常退出也使用同一清理路径。

`OpenVINS` 保留原 VIO 发布锁和位姿门控；`PNGuidance` 保留 `MissionService`
的任务状态机与指令仲裁；`YoloProcessDetector` 继续通过独立 `detect_rt` 子进程
运行检测，`detect()` 只轮询监督状态，主程序不链接推理库。

## 生命周期

工厂和产品的构造不启动设备或线程；产品 `init()` 才启动，失败抛出异常。
通信、IMU 和相机端口由宿主拥有，必须比所有工厂和产品存活更久。
每次创建得到独立产品；共享数据源仍只能由一个运行中的消费者使用。

独立使用产品时，应先 `guidance->setVIO(vio)`，再
`detector->setGuidance(guidance)`，然后按 VIO、制导、检测的顺序初始化。
制导拒绝空 VIO，初始化后不能替换 VIO；检测启动后不能替换制导。
产品以共享所有权保留依赖，工厂销毁不影响产品存活。

重复 `init()` 不重复启动，重复 `stop()` 安全，析构自动停止。
制导停止时保留检测仍在使用的目标通道；重启制导前必须先停止检测。
发布锁不能比 VIO 产品存活更久，持有发布锁期间不能停止 VIO。

## 验证

```bash
cmake -S . -B build
cmake --build build --target main test_factory_core -j4
ctest --test-dir build -R '^(factory_core|app_config|main_firmware)$' --output-on-failure
```

工厂回归使用内存输入和轻量检测子进程，不连接设备，覆盖无参工厂创建、
依赖接线与替换保护、生命周期、所有权、制导重启和检测目标 fd 传递。
入口回归覆盖配置检查、飞控固件门控和相机连接失败路径。
