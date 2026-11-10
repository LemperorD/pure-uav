# VIO 估计器（单目 + IMU + 深度，ROS-free）

> 当前代码与验收以 [2026-11-01 集成修复记录](docs/INTEGRATION_STATUS_20261101.md) 为准。通信 patch 02/05 已合入源码，无需重复应用。

基于 OpenVINS v2.7.0（mono MSCKF + IMU）改的视觉惯性里程计。输入相机图像 + 飞控 IMU，
输出 `LOCAL_FRD` 位置/姿态/速度，经 MAVLink `ODOMETRY` 发给 ArduPilot。

实时相机输入已改为外部 `camera_server` 的共享内存通道；先启动相机服务，再运行 VIO。
现行接口与参数迁移见 [VIO 接入 camera_server](../../../assets/docs/app/camera-ipc.md)。

## 目录

| 路径 | 作用 |
|---|---|
| `apps/vio_live_main.cpp` | **实时程序**：相机+飞控线程、状态行、录制、RTSP 无关的自带查看器 |
| `apps/live_view.hpp` | 自带窗口：图像+特征、状态面板、底部 2D 俯视轨迹带 |
| `apps/vio_offline_main.cpp` | **离线回放**：同一份数据可复现实时结果（逐字节相同） |
| `src/vio_estimator.cpp` | 主入口：包 OpenVINS，喂数据、深度注入、健康门限、LOST 后重锚定 |
| `src/shm_camera_source.cpp` | 实时 VIO 共用的共享内存输入、尺寸/标定校验与服务故障检测 |
| `src/orbbec_camera.cpp` | 直采工具使用的相机驱动：自动测光（含运行中重测光）、深度流+激光重开、帧集模式、设备时间戳 |
| `src/mavlink_link.cpp` | 飞控链路：收 `HIGHRES_IMU`@200Hz，发 `ODOMETRY`（质量/协方差/reset_counter），收 `LOCAL_POSITION_NED` 做第二意见 |
| `src/vision_pose_convert.cpp` | **坐标换算唯一地点**：OpenVINS global/IMU → LOCAL_FRD/机体系（含杆臂、yaw 对齐） |
| `src/fcu_clock.cpp` | 主机／飞控时钟偏移估计与 VIO Unix 时间换算 |
| `src/vio_health.cpp` | OK/DEGRADED/LOST 状态机（失效即停发） |
| `src/pose_logger.cpp` | CSV / TUM 输出 |
| `src/offline_dataset.cpp` | 离线数据集读取（`images.csv` + `imu.csv` + `depth/`） |
| `include/pure_uav/*.hpp` | 以上模块的公开头文件 |
| `third_party/open_vins/` | 打过补丁的 OpenVINS（深度初始化特征、诊断、协方差钳位）**必须保留** |
| `config/gemini_1920x1080/` | **生产配置**：OpenVINS 参数 + kalibr 相机/IMU 标定链 |
| `config/gemini_640x480/` | 60fps 配置：内参用 SDK 实测值，畸变沿用同一套 |
| `scripts/run_vio_live.sh` | 一键启动（自动选最新二进制、强制写 `console.log`、生成 `run_meta.txt`） |
| `scripts/eval_accuracy.py` | 闭环误差 + 路径长度合理性检查 |
| `scripts/depth_closure.py` | **精度尺子**：深度点云 ICP 当独立真值（多初值 + 正反向 + 稳定性判据） |
| `scripts/analyze_live.py` `run_vio_offline.py` `vio_rviz.sh` | 分析 / 离线 / RViz |
| `scripts/merge_into_repo.sh` | 同步进主仓库（排除数据与产物） |
| `tools/*.py` | 数据转换与诊断：`session_to_dataset` `bag_to_dataset` `tum_to_rviz` `drift_report` `plot_traj` `track_check` `compare_traj` `check_frames` |
| `tests/` | 时基与坐标换算的单元测试 |
| `rviz/vio.rviz` | RViz 布局 |
| `docs/` | 诊断记录、精度方法、RViz 用法 |
| `patches/` | 对通讯模块 IMU 回调的补丁 |

## 时间戳

在线 `vio_live` 和集成 `VioRuntime` 统一使用 **机载电脑的 Unix 微秒时间戳**（`uint64_t`）：

- IMU 保留飞控原始采样时间用于去抖、断流和重启检测，再通过 `FcuClock` 换算为 Unix 微秒。
- 相机继续使用现有主机／设备时钟到飞控的映射，与 IMU 对齐后使用相同的 Unix 偏移；深度保留与彩色帧的设备时间差。
- 估计器输入、`VisionPose.timestamp_us`、MAVLink `ODOMETRY.time_usec`、位姿／IMU 日志和在线录制数据均使用这个时间基。
- 离线回放继续沿用数据集时间戳，兼容已有 boot 时间数据集和新的 Unix 时间数据集。

时间来源与独立 detect 一样是主机 `system_clock`，运行前请确保机载电脑日期时间正确。
Unix 偏移在映射就绪后首次换算时锁定，当前时间基内保持固定，避免系统校时或串口接收抖动改变采样间隔；
飞控重启、重连或 IMU 中断触发时间基重建后重新锁定。映射未就绪或过期时不喂入未换算的数据。
超时、心跳和数据新鲜度仍使用单调时钟；这项修改不改变 detect 和制导的时间约定。

## 编译

依赖：OpenCV、CMake ≥3.16、C++17、Orbbec SDK（`/usr/local/lib/libOrbbecSDK.so`）、MAVLink 头文件。

```bash
cd /home/kxy/桌面/pure-uav-vio/src/vio/estimator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc) --target vio_live vio_offline
```

产物：`build/bin/vio_live`、`build/bin/vio_offline`。

## 跑

**1) 实时 + 录制**（默认跑 `config/gemini_1920x1080`）

```bash
cd /home/kxy/桌面/pure-uav-vio/src/vio/estimator
./scripts/run_vio_live.sh --out out/run1 -- \
  --use-depth-init --init-vel-zero \
  --record datasets/run1
```

**2) 实时 60fps（先将相机服务配置为 640×480@60，并使用匹配标定）**

```bash
./scripts/run_vio_live.sh --out out/run2 -- \
  --width 640 --height 480 --fps 60 --config-dir config/gemini_640x480 \
  --cam-ts device \
  --use-depth-init --init-vel-zero
```

**3) 离线回放**（`--init-threads 1` 才能复现）

```bash
./build/bin/vio_offline --dataset datasets/run1 --config-dir config/gemini_1920x1080 \
  --use-depth-init --init-vel-zero --init-threads 1 --tum-out out/run1/traj.tum
```

**4) 精度**

```bash
python3 scripts/eval_accuracy.py --tum out/run1/traj_safe.tum
python3 scripts/depth_closure.py --dataset datasets/run1 --tum out/run1/traj_safe.tum --pairs 0:last
```

**5) RViz**

```bash
./scripts/vio_rviz.sh out/run1/traj.tum live     # 边跑边看
./scripts/vio_rviz.sh out/run1/traj.tum replay   # 回放
```

## 常用参数

| 参数 | 说明 |
|---|---|
| `--width/--height/--fps` | 彩色分辨率与帧率（1080p 最高 30fps，640×480 可到 60fps） |
| `--config-dir` | 标定/参数目录，默认 `config/gemini_1920x1080` |
| `--aggregate full\|color` | 帧集聚合。`color`=彩色驱动、深度可选（才能 60fps） |
| `--cam-ts host\|device` | 相机时间戳来源。`device` 配合 `--aggregate color` 用（抖动 13ms→0.7ms） |
| `--use-depth-init` | 用深度给特征初始化 3D 位置（**可用性的关键**） |
| `--init-vel-zero` | 初始化静止时把初速度强制置 0 |
| `--no-depth` | 不开深度流（排查帧率上限用） |
| `--no-send` | 不发飞控（手持演示） |

## 已知限制

- 曝光 10.4ms、场景深度 2m 时，平移包络约 0.4–1.0 m/s（模糊受限）；缩短曝光可成倍放宽。
- 深度比彩色旧 17–35ms（无硬件帧同步），代码只接受 ≤35ms。
- `HIGHRES_IMU` 实测最高 200Hz（请求 400Hz 会得到更差的时间戳）。
- 精度口径为"深度几何一致性参考"，非外部真值；实测约 1–2%（路径长比例）。
