#pragma once

#include <cstdint>

namespace pure_uav {

// ---------- 飞控回传数据 ----------
// 机体姿态（rad）
struct Attitude {
    uint32_t time_boot_ms = 0;  // 飞控开机时间（毫秒）
    float roll = 0.0F;          // 机体FRD相对初始FRD的姿态
    float pitch = 0.0F;
    float yaw = 0.0F;
    float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};  // w x y z
};

// 电池电量
struct Battery {
    float voltage = 0.0F;   // 电压（V）
    int8_t remaining = -1;  // 剩余百分比（-1 表示未知）
};

// 飞控局部位置 + 速度（初始FRD）
struct LocalPosition {
    uint32_t time_boot_ms = 0;              // 飞控开机时间（毫秒）
    float x = 0.0F, y = 0.0F, z = 0.0F;     // 位置（m）
    float vx = 0.0F, vy = 0.0F, vz = 0.0F;  // 速度（m/s）
};

// 飞控回传 IMU（角速度 + 加速度，机体FRD）
struct ImuData {
    uint64_t timestamp_us = 0;              // 该帧 IMU 时间戳（微秒，飞控 boot 时间）
    float gx = 0.0F, gy = 0.0F, gz = 0.0F;  // 角速度（rad/s）
    float ax = 0.0F, ay = 0.0F, az = 0.0F;  // 加速度（m/s²）
};

// 飞控时间戳（用于 VIO 时间同步）
struct SystemTime {
    uint64_t time_unix_usec = 0;  // Unix 时间戳（微秒）
    uint32_t time_boot_ms = 0;    // 上电时间（毫秒）
};

// 飞控实际落地反馈（MAVLink EXTENDED_SYS_STATE），主机单调时钟用于新鲜度检查。
struct LandedState {
    uint8_t state = 0;  // 0=undefined, 1=on ground, 2=in air, 3=takeoff, 4=landing
    int64_t received_ns = 0;
};

// ---------- 上位机下发数据 ----------
// 位姿/速度的参考坐标系
enum class VisionFrame : uint8_t {
    LOCAL_NED = 1,   // 世界系 NED（X北 Y东 Z下）
    BODY_FRD = 12,   // 机体 FRD（X前 Y右 Z下）
    LOCAL_FRD = 20,  // 初始化 FRD
};

// VIO 位姿
struct VisionPose {
    uint64_t timestamp_us = 0;                           // us
    VisionFrame frame_id = VisionFrame::LOCAL_FRD;       // 位置的坐标系（惯性系）
    VisionFrame child_frame_id = VisionFrame::BODY_FRD;  // 速度的坐标系（机体系）
    float x = 0.0F;                                      // m
    float y = 0.0F;
    float z = 0.0F;
    float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};  // w x y z  机体系相对惯性系，由上面选的参数指定
    float vx = 0.0F;                        // m/s
    float vy = 0.0F;
    float vz = 0.0F;

    //=======================================================================
    // 【2026-10-28 联调新增】给飞控的附加信息（MAVLink ODOMETRY 里的
    // quality / reset_counter / 协方差）。默认值保持"旧行为"：不给就按未知发，
    // 所以老调用方（只填上面 10 个字段）不需要改一行代码。
    //
    //   quality       0-100 可信度。0 = 未知（负值按 0 处理）。
    //                 ArduPilot 会用 VISO_QUAL_MIN 丢弃低质量帧（默认 0，但通常
    //                 会配成 50 做保护）—— 所以"健康/降级"必须真的写进来。
    //   reset_counter 估计器重建（重锚定）计数，每次递增并按 256 回绕。飞控在其变化时
    //                 触发位置/速度/姿态重锚定；不能拿质量或别的量冒充。
    //   pos_sigma_m / vel_sigma_mps  标准差（米 / m/s）。<=0 表示未知，
    //                 发送时协方差对应元素填 NaN（MAVLink 约定 NaN = 未知）。
    //=======================================================================
    int8_t quality = 0;
    uint8_t reset_counter = 0;
    float pos_sigma_m = 0.0F;
    float vel_sigma_mps = 0.0F;
};

// 制导控制模式
enum class GuidanceMode : uint8_t {
    POSITION = 0,                  // 位置控制（发 x/y/z）
    VELOCITY = 1,                  // 速度控制（发 vx/vy/vz）
    POS_WITH_VEL_FEEDFORWARD = 2,  // 位置目标 + 速度前馈
    ACCELERATION = 3,              // 加速度（发 afx/afy/afz）
};

// 制导指令：期望状态（NED）。制导律（比例导引）只输出加速度，不输出航向。
struct GuidanceCmd {
    GuidanceMode mode = GuidanceMode::VELOCITY;  // 控制模式
    float x = 0.0F;                              // 期望位置（mode=POSITION 时有效）
    float y = 0.0F;
    float z = 0.0F;
    float vx = 0.0F;  // 期望速度（mode=VELOCITY 时有效）
    float vy = 0.0F;
    float vz = 0.0F;
    float afx = 0.0F;  // 期望加速度（mode=ACCELERATION 时有效）
    float afy = 0.0F;
    float afz = 0.0F;
};

// ---------- 飞行模式常量 ----------
// PX4 的 custom_mode 值，作为 sendSetMode(uint32_t) 的输入
namespace px4_mode {
constexpr uint32_t MANUAL = (0U << 24) | (1U << 16);        // 手动
constexpr uint32_t ALTCTL = (0U << 24) | (2U << 16);        // 定高
constexpr uint32_t POSCTL = (0U << 24) | (3U << 16);        // 定点
constexpr uint32_t ACRO = (0U << 24) | (5U << 16);          // 特技
constexpr uint32_t OFFBOARD = (0U << 24) | (6U << 16);      // 外部控制
constexpr uint32_t STABILIZED = (0U << 24) | (7U << 16);    // 自稳
constexpr uint32_t ALT_CRUISE = (0U << 24) | (11U << 16);   // 高度巡航
constexpr uint32_t AUTO_READY = (1U << 24) | (4U << 16);    // 自动-就绪
constexpr uint32_t AUTO_TAKEOFF = (2U << 24) | (4U << 16);  // 自动-起飞
constexpr uint32_t AUTO_LOITER = (3U << 24) | (4U << 16);   // 自动-悬停
constexpr uint32_t AUTO_MISSION = (4U << 24) | (4U << 16);  // 自动-任务
constexpr uint32_t AUTO_RTL = (5U << 24) | (4U << 16);      // 自动-返航
constexpr uint32_t AUTO_LAND = (6U << 24) | (4U << 16);     // 自动-降落
}  // namespace px4_mode

// ArduPilot ArduCopter 飞行模式，作为 sendSetMode(uint32_t) 的输入
namespace arducopter_mode {
constexpr uint8_t STABILIZE = 0;  // 自稳
constexpr uint8_t ACRO = 1;       // 特技
constexpr uint8_t ALT_HOLD = 2;   // 定高
constexpr uint8_t AUTO = 3;       // 自动任务
constexpr uint8_t GUIDED = 4;     // 引导（离板控制，对应 PX4 Offboard）
constexpr uint8_t LOITER = 5;     // 悬停
constexpr uint8_t RTL = 6;        // 返航
constexpr uint8_t CIRCLE = 7;     // 环绕
constexpr uint8_t LAND = 9;       // 降落
constexpr uint8_t POSHOLD = 16;   // 定点
constexpr uint8_t SMARTRTL = 21;  // 智能返航
}  // namespace arducopter_mode

}  // namespace pure_uav
