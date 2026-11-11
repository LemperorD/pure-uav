# VIO 接入 camera_server

更新：2026-11-10。本文描述合并 detect 与 VIO 分支后的相机接口和进程分工。
整机模块分工见 [usage.md](usage.md)；无硬件测试不替代整机实机联调。
`camera_server` 独占相机，当前仓库的 `main` 和 `vio_live` 均通过 POSIX 共享内存读图，
不再创建 Orbbec 采集管线。

## 数据路径

```text
Orbbec → camera_server → /pure_uav_camera_vio → main 或 vio_live
                       /pure_uav_camera_color → 独立检测进程
飞控 IMU → main 的 Communicator 或 vio_live 的 MavlinkLink → VIO → ODOMETRY
```

`main` 与 `vio_live` 是两种替代入口，各自占用飞控串口，应选择其中一种运行。
`main` 内运行通信与 VIO，并在心跳、相机通道和 VIO 就绪后启动独立 `detect_rt` 子进程。
检测通过彩色通道读图，独立监督线程负责重启与回收，不阻塞 VIO 主循环。
使用 `--no-detection` 可仅运行通信和 VIO；检测参数见 `config/detector-rt.yaml`。

协议头和通道实现位于 `src/common/camera_frame.hpp`、`camera_shm.hpp/.cpp`，
与接口提供方保持相同布局和 `kShmVersion = 1`。读者只读映射、不删除服务端通道。
`ShmCameraSource` 位于 `src/vio/estimator/src/shm_camera_source.*`，供两个入口共用。

## 启动

三个程序均从本仓库构建。先运行配置校验，再启动相机服务，等待报告“两通道可连接”后
在另一个终端启动 `main`：

```bash
cmake --build build --target camera_server main vio_live detect_rt -j2
./bin/camera_server --check-config
./bin/detect_rt --check-config
./bin/main --check-config
# 终端 A：
./bin/camera_server --config config/camera.yaml
# 终端 B：VIO 和检测一起运行，但不发送位姿
./bin/main --no-send
# 仅检查 VIO 时改用：./bin/main --no-send --no-detection
```

`main` 使用现有 `config/driver.yaml` 和生产标定 `gemini_1920x1080`，
期望服务端提供 1920×1080@30。也可传入 `--config-dir` 选择标定，分辨率从标定读取。
`main` 的运行参数来自 `config/vio.yaml`；当前文件启用深度初始化和静止起步速度约束。
修改 `use_depth`、`use_depth_init`、`init_vel_zero` 等字段后，使用 `main --check-config`
确认实际值。配置分工与路径解析见 [usage.md](usage.md#vio-配置)。

独立调试、录制和深度初始化使用 `vio_live`，例如：

```bash
./bin/vio_live --serial /dev/ttyACM0 --baud 921600 \
  --config-dir src/vio/estimator/config/gemini_1920x1080 \
  --width 1920 --height 1080 --fps 30 \
  --use-depth-init --init-vel-zero --no-send
```

两种入口均支持 `--shm-vio NAME` 和 `--shm-timeout-ms N`，默认通道
`/pure_uav_camera_vio`，默认等待 5000 ms，超时参数允许 0–60000 ms。
自定义通道必须与服务端 `channels.vio_name` 或 `--vio-channel` 一致。

曝光、增益、白平衡、测光、聚合和深度采集开关统一在服务端 `config/camera.yaml` 设置。
`vio_live` 的旧采集参数（如 `--exposure`、`--aggregate`、`--tune-expected-rate`）
会报出迁移提示；`--no-depth` 仅让该消费者忽略深度图，不会改变服务端采集。

## 进程与通道

两条独立通道使用 seqlock 三缓冲，慢消费者不占用另一通道槽位。VIO 默认 3×8 MiB，
彩色默认 3×4 MiB；`/dev/shm` 至少预留 40 MiB。写者不等待消费者，消费者只读最新稳定帧。
`camera_server` 创建并在正常关闭时清理通道；消费者关闭不执行 `shm_unlink`。

`main --detect-bin FILE` 覆盖检测可执行程序，默认取 `main` 同目录的 `detect_rt`。
`--detection-config FILE` 把配置路径传给检测进程；`--detect-arg ARG` 可重复传递运行参数。
检测算法配置由子进程自行读取，`main` 不加载推理模型。异常退出最多重启 3 次；
正常退出或重启超限时主程序结束，并停止 VIO。Ctrl-C、相机/VIO 故障或异常退出均先回收检测进程。
相机服务由外部托管，停止 `main` 不停止服务。独立使用 `vio_live` 时，检测也可单独启动。

## 帧与时间

- 读取最新完整帧，跳过的序号计入丢帧数；返回的图像自有内存，不受后续发布覆盖影响。
- 灰度必须恰好为 `width × height` 字节，深度必须匹配原始深度尺寸和 `CV_16UC1` 长度。
- 流分辨率必须同时匹配消费者期望与标定链，帧率也必须匹配期望，否则启动失败。
- 深度使用通道传入的内参及彩色到深度外参，平移从毫米转米；需要深度时缺失或非法即启动失败。
- `host_ns` 保留服务端采集时的 `CLOCK_MONOTONIC`，不以消费者收到共享内存的时刻替换。
- `device_us` 和 `depth_device_us` 保留同源设备时间，用于深度相对时差；
  `sys_us` 单独保留，供 `vio_live --cam-ts device` 维持原驱动的 SDK 系统时间平滑行为。
- 缺失或非法深度时间戳时，`vio_live` 跳过深度注入，不伪造彩色时间作为深度时间。
- 飞控时钟映射、IMU 覆盖检查和最终 Unix 微秒输出继续沿用现有 VIO 逻辑。

服务关闭、超过 2 秒无发布心跳、帧结构非法、相机会话或代数变化时，输入停止并报错。
恢复顺序为先启动相机服务，再重启 VIO 消费者，使滤波器与时间基重新初始化。
停止消费者不会关闭相机服务。直采标定工具或 `depth_range_probe` 应在相机服务停止后使用。

## 无硬件验证

```bash
cmake --build build --target test_camera_shm test_shm_camera_source -j2
ctest --test-dir build --output-on-failure -R '^(camera_shm|shm_camera_source)$'
```

共享内存测试覆盖跨进程读取、双通道隔离、seqlock 一致性与清理；VIO 输入测试覆盖
标定/尺寸校验、灰度和深度恢复、时间戳保持、去重、丢帧、丢弃旧帧、停止/重启、
会话变化及非法负载拒绝。测试不打开相机和飞控，不代表实机定位精度或 EKF 融合验收。

检测配置和生命周期回归为 `detector_config`、`detection_process`、`detection_supervisor`；
后者验证检测监督不阻塞宿主、异常重启有界，以及宿主退出时子进程不会残留或成为僵尸。
