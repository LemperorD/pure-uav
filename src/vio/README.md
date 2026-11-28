# VIO

基于仓库内的 OpenVINS，用单目图像、飞控 IMU 和可选深度估计位姿；运行不依赖 ROS。
机载 `main` 集成 VIO，`vio_live` 用于实时调试和录制，`vio_offline` 用于回放。

- `apps/`：实时、离线入口；`src/` 和 `include/`：估计器与运行时。
- `config/`：运行参数和相机／IMU 标定链，包含 `gemini_640x480/` 和 `gemini_1920x1080/`。
- `scripts/`、`tools/`、`tests/`：启动、分析和测试；`third_party/open_vins/` 为构建依赖。

相机内参、相机–IMU 外参、IMU 噪声和加速度标定工具已迁至 `calibration_for_vio`。
本仓库保留运行所需的标定 YAML，构建和部署不需要标定工具或额外应用补丁。

## 编译

依赖 C++17、Eigen3、OpenCV 4、Boost filesystem/date_time、Ceres；实时构建使用仓库内的 Orbbec SDK 和 MAVLink。
从仓库根目录运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target main camera_server vio_live vio_offline
```

产物位于 `bin/`。只构建 VIO 时也可用 `cmake -S src/vio -B build/vio`，产物位于 `build/vio/bin/`。
独立构建可用 `-DPURE_VIO_BUILD_LIVE=OFF` 跳过实时目标；实时运行仍需要整仓构建的 `camera_server`。
若独立构建以前使用了旧源码路径，请换一个构建目录，不能复用旧 CMake 缓存。

## 运行

机载启动与相机参数见 [相机服务接入](../../assets/docs/app/camera-ipc.md)。单独调试时，先启动相机服务，再从仓库根目录运行：

```bash
bash scripts/run_vio_live.sh --out out/run1 -- --record datasets/run1
./bin/vio_offline --dataset src/vio/datasets/run1 \
  --config-dir src/vio/config/gemini_1920x1080 --init-threads 1 \
  --tum-out src/vio/out/run1/replay.tum
```

实时脚本默认不向飞控发送位姿，输出和录制路径相对 `src/vio/`。
在线默认使用 `config/camera.yaml` 的 640×480 配置。切换到 1920×1080 时，给脚本传入 `-- --camera-config config/camera_1920x1080.yaml`。
在所选相机配置中调整帧率、深度开关或在线标定。在 `config/driver.yaml` 中修改 IMU 请求间隔。
实时入口可分别用 `--camera-config` 和 `--driver-config` 指定其他配置文件；离线回放仍按数据集选择 `--config-dir`。
分析工具的 Python 依赖见 [requirements.txt](requirements.txt)；RViz 另需 ROS 2。
使用直采曝光诊断时，将 `VIO_CALIBRATION_REPO` 指向 `calibration_for_vio` 的仓库根目录，并先停止相机服务。

## 时间戳

在线估计器输入、`VisionPose.timestamp_us`、`ODOMETRY.time_usec` 和录制数据统一使用主机 Unix 微秒。
相机和 IMU 先对齐飞控采样时基，再转换；超时和心跳仍使用单调时钟。离线回放沿用数据集的时间戳。
参见 [系统时间基](../../assets/docs/decisions/0005-time-base.md) 和 [实时调试笔记](LIVE_DEBUG_NOTES.md)。
