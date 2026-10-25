// VisionPose / VisionFrame 定义。
//
// 这里与「飞控通信」同学在 driver/communicate/include/communicator.hpp 中的
// 定义保持逐字节一致（字段顺序、类型、默认值、枚举值都相同），这样 VIO 模块
// 可以直接把结果交给 pure_uav::Communicator::pushVisionPose()，不需要再做转换。
//
// 坐标系约定（与 MAVLink 20/12 一致）：
//   frame_id       = LOCAL_FRD：位置/姿态的参考系。原点 = VIO 初始化时刻的机体
//                    位置；x = 初始化时机头在水平面的投影，y = 右，z = 重力向下。
//   child_frame_id = BODY_FRD ：速度的参考系，即当前机体系（x 前、y 右、z 下）。
//   q              ：机体系相对 LOCAL_FRD 的姿态，w x y z。
//   timestamp_us   ：该位姿对应的时刻。在线运行时使用飞控 boot 时间（与
//                    HIGHRES_IMU.time_usec 同一时钟域）。

#pragma once

#include <cstdint>

// 集成到 pure-uav 仓库后，如果编译单元能看到 driver/communicate 的
// communicator.hpp，就直接复用通信模块的定义（避免同一个 struct 定义两份）。
#if defined(__has_include)
#if __has_include("communicator.hpp")
#define PURE_UAV_HAVE_COMMUNICATOR_HEADER 1
#include "communicator.hpp"
#endif
#endif

#ifndef PURE_UAV_HAVE_COMMUNICATOR_HEADER
namespace pure_uav {

// 位姿/速度的参考坐标系
enum class VisionFrame : uint8_t {
    LOCAL_NED = 1,   // 世界系 NED（X北 Y东 Z下）
    BODY_FRD = 12,   // 机体 FRD（X前 Y右 Z下）
    LOCAL_FRD = 20,  // 世界系 FRD
};

// VIO 位姿
struct VisionPose {
    uint64_t timestamp_us = 0;  // us，飞控 boot 时间（在线）
    VisionFrame frame_id = VisionFrame::LOCAL_FRD;        // 位置的坐标系（惯性系）
    VisionFrame child_frame_id = VisionFrame::BODY_FRD;   // 速度的坐标系（机体系）
    float x = 0.0f;     // m
    float y = 0.0f;
    float z = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z  机体系相对惯性系
    float vx = 0.0f;  // m/s
    float vy = 0.0f;
    float vz = 0.0f;

    // 【2026-10-21】诊断字段（不参与和飞控的通信，只写进 CSV）：
    // 当前滤波器内部的 cam-IMU 时延估计值（秒）。只值在 estimate_camera_timeoffset 打开时变化。
    // 复核要求把随时间变化的 td 记录下来 —— 否则无法判断在线时延估计是否跑飞。
    float cam_imu_dt = 0.0f;
};

}  // namespace pure_uav

#endif  // PURE_UAV_HAVE_COMMUNICATOR_HEADER
