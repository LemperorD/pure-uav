# ADR 0006：相机 / VIO 与制导的边界

- 状态：现行（VIO 已合并，2026-11-11 接入检测同帧深度测距）
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/flight/guidance_loop.cpp`（像素反投影）、`GuidanceLoopConfig::cam`
- 相关文档：[interfaces.md](../interfaces.md)

## 1 相机模型

`GuidanceCameraModel`：

- `fx / fy`：焦距（像素）；
- `cx / cy`：主点；
- `R_body_cam[9]`：相机 → 机体（PX4 FRD）旋转，行主序；
- `valid()`：`fx>0 && fy>0`。

像素 → 视线：

```text
d_cam   = unproject_radtan(u, v, intrinsics, distortion)
d_body  = R_body_cam · d_cam
d_world = R(q) · d_body
los_hat = normalize(d_world)
```

- `q` 来自 `GuidanceInsSample`（机体 → 世界）；
- `R_body_cam` 的目标系是 FRD；
- 目标在相机后方或结果零矢量 → 视作目标丢失。

## 2 标定

- 内参：`src/vio/extrinsic/results/final/camchain-openvins-final.yaml`；
- 外参：`src/vio/extrinsic/results/final/final_extrinsics.json`；
- 这些结果已在当前仓库；生产 RGB 内参另见 `src/vio/estimator/config/gemini_1920x1080/kalibr_imucam_chain.yaml`；
- 演示用 `guidance_demo_camera()`（f=600、主点 320×240）只是占位，**真机不能直接用**。

真机 `main --mission` 单独读取 `config/guidance-camera.yaml`；当前安装外参未确认，`calibrated=false` 时只搜索。
独立仿真样例仍可回退 `guidance_demo_camera()`。深度测距与实际标定校验见 [depth-guidance.md](../app/depth-guidance.md)。

## 3 VIO 输出

VIO 的 `VisionPose`（与 communication 的 `communicator.hpp` 字段一致）：

- `frame_id = LOCAL_FRD`，`child = BODY_FRD`；
- 原点 = 初始化位置；
- x = 初始化时机头水平投影；
- z = 重力向下；
- `timestamp_us` = FC boot µs；
- 通过 MAVLink `ODOMETRY` 发给 PX4（quality/reset/cov 由 VIO 填）。

## 4 推荐接入方式

**让 PX4 融合 VIO，制导继续读 PX4。**

```text
VIO → MAVLink ODOMETRY → PX4 EKF2 EV → vehicle_local_position → GuidanceLoop
```

- 制导代码不改；
- `GuidanceInsSample` 仍来自飞控 local position/attitude；
- 需要设置 PX4 参数：
  - `EKF2_EV_CTRL`（启用水平/垂直/速度/yaw 融合）；
  - `EKF2_EV_DELAY`（ms，补偿链路/曝光延迟）；
  - `EKF2_EV_POS_X/Y/Z`（VIO 相对飞控的杆臂）；
  - 无 GPS 时确认 `EKF2_GPS_CTRL` 与 arming 检查。

**不建议**让制导直接消费 VIO：PX4 offboard 仍需要飞控自身有有效位姿。

## 5 时间与坐标系对齐

- VIO 初始化与 PX4 EKF 初始化必须同一姿态，否则差固定 yaw；
- `ODOMETRY` 的 `time_usec` 必须是观测时刻，不是发送时刻；
- 链路延迟用 `EKF2_EV_DELAY` 补偿；
- VIO 的 LOCAL_FRD 与 PX4 无 GPS local 系数值上一致（初始化机头为 x、z 下）。

## 6 维护

- 换相机/重标定 → 更新 YAML + 本文；
- VIO 合并后 → 把 `results/final` 带进主分支，或在文档中明确“本地生成”；
- 改 `R_body_cam` 约定 → 同步 `interfaces.md` §3 和 §7；
- 改 VIO 接入方式 → 更新本文和 `architecture.md`。
