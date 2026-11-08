# 单进程 VIO 主程序第一版

后续更新：可选检测已接入，默认配置仍关闭检测；详见 detection-integration.md。
本周上机步骤见 uav-vio-bench-checklist.md。下方第一版 14 项测试记录保留为历史基线，
检测接入后的最新结果以 build/verification-detection.json 为准。
后续遗留问题修复与全量验证见 acceptance-fixes.md，最新全量日志位于 build-fix/。

实现位置：`src/app/main.cpp`。本版本只做飞控 IMU + 相机 → VIO → MAVLink ODOMETRY。
工作副本为 `/home/kxy/桌面/pure-uav-test/pure-uav-main`，原工程和标定文件不改动。

## 构建与无硬件检查

```bash
cd /home/kxy/桌面/pure-uav-test/pure-uav-main
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target main vio_live -j2
./bin/main --help
./bin/main --check-config
```

`--config` 默认 `config/app.yaml`，`--driver-config` 默认 `config/driver.yaml`。
相对路径以本副本的仓库根目录解析，从 `/tmp` 启动也可以；支持显式绝对配置路径。
`--help` / `--check-config` 不创建相机实例、不打开串口。
配置错误返回 2，启动/运行错误返回 1，信号退出返回 0。
本入口只接受 ArduPilot 心跳；不会发送解锁、起降或切模式命令。

## 线程与所有权

- 主线程按 `steady_clock` 的固定节拍运行，默认 50 Hz；跳过错过的节拍，不补发突发流。
- 一个 Communicator 独占串口。内部线程解析 MAVLink、维护连接与逐帧 IMU 队列。
- CameraService 独占相机。回调复制灰度/原始深度图到容量 5 的队列，满时丢最旧并计数。
- VioRuntime 工作线程独占 VioEstimator；顺序处理 IMU/图像并发布线程安全快照。
  主线程不直接读取估计器内部的健康、初始化和重建状态。
- 正常退出顺序：停止相机 → 停止并 join VIO → 停止通信。

## 定位发布

主循环只发送新且有效的定位，不把重复读取缓存当作新定位。
时间戳小于或等于上次发送时间均拒绝，连接会话或时间基变化后重新起算。
检查健康、时钟、连接、会话、时间基代数、时间戳单调性、数值与四元数、坐标系和数据年龄。
默认采集年龄和结果产生年龄均必须 ≤150 ms，边界包含在内；未来/未知时间也拒绝。
采集年龄包含图像解码、队列等待及估计耗时，不能用刚完成的处理时间掩盖旧图像。

发布检查和串口发送之间持有一个短期发布锁，时间基重置不能越过这次发送。
Communicator 在实际串口写入前，持发送锁再次检查期望会话；连接失效与会话变更使用同一把锁。
非阻塞发送失败只记数，不在当前 tick 忙等重试；成功后才推进 last_sent_us。
串口写入成功仅说明本地写入完成，不能代表飞控收到或已融合。

ODOMETRY 保持 LOCAL_FRD(20) / BODY_FRD(12)、wxyz 和原始定位时间戳。
健康时质量/位置 sigma/速度 sigma 默认为 90 / 0.15 m / 0.25 m/s；降级时为 50 / 0.5 m / 0.8 m/s。
这些是配置的质量策略，并非这次测得的真实误差或估计器完整协方差。
UNKNOWN / LOST 停发。重建计数按 uint8 回绕填入 reset_counter。
地面未解锁也发送定位；armed 只控制零速更新，地面开、解锁后关。

## 时间与恢复

使用主机单调到达时间经 FcuClock 映射到飞控 IMU 时间，本轮未切换为设备时间给图像打时标。
启动和恢复时丢弃 150 个 IMU 样本；去抖步长由实际 IMU 时间戳频率测量决定，非请求频率。
只处理具有 IMU 历史并已覆盖到 `t_camera + camera_timeoffset` 的图像；等待覆盖最多 100 ms。
会话变化、时钟跳变、IMU 间隔超过 100 ms、输入队列溢出时停发、递增代数并重建。
清除旧时间基的 pending 图像、时钟、去抖和频率窗口，拒绝早于恢复边界的队列样本。
时钟与估计器重新准备好后，只有新的健康快照才允许再次发布。

相机原回调把 SDK 系统时间传到名为 device_us 的参数。新 CameraService 显式请求真实设备时间，
仅用于计算同源的彩色/深度时间差；旧 vio_live 保留其默认回调行为。
深度使用原始深度相机内参及彩色→深度外参，平移由 mm 转 m。
深度几何不合法时拒绝启动，不能把零旋转矩阵当作有效外参。
深度时间戳缺失或相差过大时不伪造时间，拒绝该深度输入并计数；可以从状态行看见。

嵌入运行模式关闭 OpenVINS 控制台热路径输出与定时文件记录；拒绝继承诊断/算法覆写环境变量。
旧 vio_live 默认日志与 CLI 保持可用。没有修改生产标定链、模型或原始测试数据。

## 状态与故障

状态行给出连接、固件、模式、会话/代数、实测 IMU Hz、健康/特征、发送/失败/拦截原因、
重置与丢帧、深度使用/拒绝数和采集年龄。工作线程异常保存 last_error，主程序检测到后退出。
相机停止、持续没有图像、VIO 线程退出均结束主程序。飞控掉线由通信线程尝试重连，期间停发。
启动等心跳有期限，等待期间信号可以取消。相机启动测光仍使用现有同步驱动实现。
VioRuntime 的估计器初始化等待尚无可中断的硬期限；这不等于心跳等待超时。
底层初始化若永久阻塞，在不强杀线程、不分离引用外部对象的线程的条件下，当前无法保证启动/停止有界。

## 无硬件回归

```bash
cmake --build build --target main vio_live test_pose_gate test_vio_runtime test_heartbeat_wait \
  test_vision_pose_wire test_fcu_clock test_live_timing test_vio_health test_pose_convert \
  test_odometry_pack test_transport_session test_communicator_session test_command_targets \
  test_link_backpressure -j2
ctest --test-dir build --output-on-failure -j2 \
  -R '^(pose_gate|vio_runtime|heartbeat_wait|app_config|vision_pose_wire|fcu_clock|live_timing|vio_health|pose_convert|odometry_pack|transport_session|communicator_session|command_targets|link_backpressure)$'
```

测试使用假输入和伪终端，不接真实设备。工作线程测试也调用真实估计器处理无特征测试图像，
检查不会凭空生成有效定位；这不等于验证有运动时的定位精度或实机融合。

## 真机台架步骤（本轮只记录，不执行）

1. 在实际机器确认飞控固件、相机/IMU 安装关系及串口路径，修改副本 driver.yaml 的设备名。
2. 先运行 `./bin/main --check-config`。确认 200 Hz IMU 请求、相机分辨率与标定匹配。
3. 拆桨并保持未解锁，在操作人员准备好后手动运行 `./bin/main`；观察测光、IMU 频率、
   初始化与健康、定位发送数、采集年龄和深度使用/拒绝数。初始化需要足够的运动激励。
4. 地面进行前/右/上/偏航动作，检查飞控日志里的视觉位置、姿态和速度轴向、质量及重置计数。
   同时检查实际接收率和 EKF 是否融合，不能只看本地 sent 增加。
5. 在当前 ArduPilot 固件中人工核对 MAVLink 视觉来源、质量门限、EKF 外部导航来源、
   误差/杆臂和无 GPS 原点配置；本程序不自动修改这些参数。
6. 在地面分别验证遮挡、相机断开、飞控链路断开和飞控重启：失效后不得继续发送旧位姿，
   恢复必须有新会话/代数与重新初始化，reset_counter 的变化应在飞控日志中可见。
7. 用 Ctrl-C 验证退出、再次启动和设备释放。台架接收/融合与失效恢复通过后再另行评估飞行。

本轮未验证真实 USB 枚举、相机曝光效果、VIO 运动精度、飞控 EKF 融合或飞行行为。

## 后续接口

主循环发布定位之后留有应用调度入口。可选检测已接入共享采集和彩色目标快照，默认仍关闭；
未来制导可以在固定节拍内消费飞控融合状态与目标快照。当前 main 不链接制导，也不发送制导指令。

通信的 getAttitude/getLocalPosition/getSystemTime 仅在本会话已收到该遥测且仍连接时成功。
原单参数调用保留；新重载支持 max_age_ms，负数表示不限制年龄，失败不修改输出。
对应 AgeMs 在本会话无数据或断线后返回 -1；不能仅凭单位四元数/零缓存推断已收有效遥测。

## 2026-11-05 软件验证结果

- x86_64 开发机：main、vio_live 和相关测试目标构建成功。项目代码无编译告警；
  MAVLink 第三方生成头的 packed-member 既有告警保留。
- 选定的 14 项无硬件回归全部通过（0 失败，2.01 s）；不是对所有仓库测试的全量声明。
- 发布门控包含 26 项检查，CLI/配置包含 18 项检查。
- 从 `/tmp` 执行 main --check-config 返回 0，找到的是测试副本的配置与标定目录。
- main --help、vio_live --help 返回 0。未运行 main/vio_live 的真实设备路径。
- ldd 无缺失依赖，不含 detection_rt / guidance；nm 无项目检测/制导符号。
- Communicator 原有 pushVisionPose(const VisionPose&) 符号保留；
  新主程序使用 pushVisionPoseForSession()，共用同一套 ODOMETRY 编码。
- 原工程全部 1015 个文件/符号链接条目与复制前基线一致，没有删除原有文件。
- 本次未在 ARM 板端构建或测量性能，未验证真实相机、串口、飞控融合和飞行。

可复核记录：
- `build/build-final.log`：最终构建日志。
- `build/ctest-final.log`：14 项测试结果。
- `build/verification.json`：原工程核对结果及新增/修改文件清单。
- `build/source-copy-diff.txt`：原工程与已实现副本的 diff -rq 输出。
- `build/main-ldd.txt`：实际运行库依赖。
- `source-baseline.json`：复制前的内容校验基线。

本轮修改的既有文件：

```text
config/driver.yaml
src/app/CMakeLists.txt
src/app/main.cpp
src/driver/communicate/include/communicator.hpp
src/driver/communicate/src/communicator.cpp
src/test/CMakeLists.txt
src/test/test_driver/test_vision_pose_wire.cpp
src/vio/estimator/CMakeLists.txt
src/vio/estimator/apps/live_view.hpp
src/vio/estimator/include/pure_uav/vio_estimator.hpp
src/vio/estimator/src/orbbec_camera.cpp
src/vio/estimator/src/orbbec_camera.hpp
src/vio/estimator/src/vio_estimator.cpp
src/vio/estimator/third_party/open_vins/ov_core/src/track/TrackKLT.cpp
src/vio/estimator/third_party/open_vins/ov_msckf/src/core/VioManager.cpp
```

本轮新增文件：

```text
assets/docs/app/single-process-vio.md
config/app.yaml
src/app/app_config.cpp
src/app/app_config.hpp
src/app/camera_service.cpp
src/app/camera_service.hpp
src/app/glue/comm_imu_source.cpp
src/app/glue/comm_imu_source.hpp
src/app/pose_gate.cpp
src/app/pose_gate.hpp
src/test/test_app/test_app_config.py
src/test/test_app/test_heartbeat_wait.cpp
src/test/test_app/test_pose_gate.cpp
src/test/test_app/test_vio_runtime.cpp
src/vio/estimator/include/pure_uav/vio_runtime.hpp
src/vio/estimator/src/vio_runtime.cpp
```
