#pragma once

#include <cstdint>

namespace pure_uav {

// ---------- 飞控回传数据 ----------
// 机体姿态（rad）
struct Attitude {
    uint32_t time_boot_ms = 0;  // 飞控开机时间（毫秒）
    float roll = 0.0f;  // 机体FRD相对初始FRD的姿态
    float pitch = 0.0f;
    float yaw = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z
};

// 电池电量
struct Battery {
    float voltage = 0.0f;    // 电压（V）
    int8_t remaining = -1;   // 剩余百分比（-1 表示未知）
};

// 飞控局部位置 + 速度（初始FRD）
struct LocalPosition {
    uint32_t time_boot_ms = 0;                    // 飞控开机时间（毫秒）
    float x = 0.0f, y = 0.0f, z = 0.0f;       // 位置（m）
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;    // 速度（m/s）
};

// 飞控回传 IMU（角速度 + 加速度，机体FRD）
struct ImuData {
    uint64_t timestamp_us = 0;                    // 该帧 IMU 时间戳（微秒，飞控 boot 时间）
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;        // 角速度（rad/s）
    float ax = 0.0f, ay = 0.0f, az = 0.0f;        // 加速度（m/s²）
};

// 飞控时间戳（用于 VIO 时间同步）
struct SystemTime {
    uint64_t time_unix_usec = 0;   // Unix 时间戳（微秒）
    uint32_t time_boot_ms = 0;     // 上电时间（毫秒）
};

// ---------- 上位机下发数据 ----------
// 位姿/速度的参考坐标系
enum class VisionFrame : uint8_t {
    LOCAL_NED = 1,         // 世界系 NED（X北 Y东 Z下）
    BODY_FRD = 12,         // 机体 FRD（X前 Y右 Z下）
    LOCAL_FRD = 20,        // 初始化 FRD
};

// VIO 位姿
struct VisionPose {
    uint64_t timestamp_us = 0;  // us
    VisionFrame frame_id = VisionFrame::LOCAL_FRD;        // 位置的坐标系（惯性系）
    VisionFrame child_frame_id = VisionFrame::BODY_FRD;   // 速度的坐标系（机体系）
    float x = 0.0f;  // m
    float y = 0.0f;
    float z = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z  机体系相对惯性系，由上面选的参数指定
    float vx = 0.0f;  // m/s
    float vy = 0.0f;
    float vz = 0.0f;
};

// 制导控制模式
enum class GuidanceMode : uint8_t {
    POSITION = 0,  // 位置控制（发 x/y/z）
    VELOCITY = 1,  // 速度控制（发 vx/vy/vz）
    POS_WITH_VEL_FEEDFORWARD = 2,  // 位置目标 + 速度前馈
    ACCELERATION = 3,  // 加速度（发 afx/afy/afz）
};

// 制导指令：期望状态（NED）。制导律（比例导引）只输出加速度，不输出航向。
struct GuidanceCmd {
    GuidanceMode mode = GuidanceMode::VELOCITY;  // 控制模式
    float x = 0.0f;    // 期望位置（mode=POSITION 时有效）
    float y = 0.0f;
    float z = 0.0f;
    float vx = 0.0f;   // 期望速度（mode=VELOCITY 时有效）
    float vy = 0.0f;
    float vz = 0.0f;
    float afx = 0.0f;  // 期望加速度（mode=ACCELERATION 时有效）
    float afy = 0.0f;
    float afz = 0.0f;
};

// ---------- 飞行模式常量 ----------
// PX4 的 custom_mode 值，作为 sendSetMode(uint32_t) 的输入
namespace px4_mode {
constexpr uint32_t MANUAL       = (0u << 24) | (1u << 16);  //手动
constexpr uint32_t ALTCTL       = (0u << 24) | (2u << 16);  //定高
constexpr uint32_t POSCTL       = (0u << 24) | (3u << 16);  //定点
constexpr uint32_t ACRO         = (0u << 24) | (5u << 16);  //特技
constexpr uint32_t OFFBOARD     = (0u << 24) | (6u << 16);  //外部控制
constexpr uint32_t STABILIZED   = (0u << 24) | (7u << 16);  //自稳
constexpr uint32_t ALT_CRUISE   = (0u << 24) | (11u << 16); //高度巡航
constexpr uint32_t AUTO_READY   = (1u << 24) | (4u << 16);  //自动-就绪
constexpr uint32_t AUTO_TAKEOFF = (2u << 24) | (4u << 16);  //自动-起飞
constexpr uint32_t AUTO_LOITER  = (3u << 24) | (4u << 16);  //自动-悬停
constexpr uint32_t AUTO_MISSION = (4u << 24) | (4u << 16);  //自动-任务
constexpr uint32_t AUTO_RTL     = (5u << 24) | (4u << 16);  //自动-返航
constexpr uint32_t AUTO_LAND    = (6u << 24) | (4u << 16);  //自动-降落
}  // namespace px4_mode

// ArduPilot ArduCopter 飞行模式，作为 sendSetMode(uint32_t) 的输入
namespace arducopter_mode {
constexpr uint8_t STABILIZE = 0;   // 自稳
constexpr uint8_t ACRO      = 1;   // 特技
constexpr uint8_t ALT_HOLD  = 2;   // 定高
constexpr uint8_t AUTO      = 3;   // 自动任务
constexpr uint8_t GUIDED    = 4;   // 引导（离板控制，对应 PX4 Offboard）
constexpr uint8_t LOITER    = 5;   // 悬停
constexpr uint8_t RTL       = 6;   // 返航
constexpr uint8_t CIRCLE    = 7;   // 环绕
constexpr uint8_t LAND      = 9;   // 降落
constexpr uint8_t POSHOLD   = 16;  // 定点
constexpr uint8_t SMARTRTL  = 21;  // 智能返航
}  // namespace arducopter_mode

}  // namespace pure_uav
