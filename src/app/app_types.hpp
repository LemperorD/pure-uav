#pragma once
// =============================================================================
//  app_types.hpp —— app 层类型 / 契约 / 配置（合并后的“公共词典”）
//
//  合并自：
//    · sample.hpp          InsSample / TargetSample
//    · output.hpp          InputHealth / ControlMode / GuidanceOutput / LoopStatus
//    · sensor_source.hpp   IInsSource / ITargetSource
//    · command_sink.hpp    ICommandSink / NullSink
//    · guidance_loop.hpp   CameraModel / LoopConfig
//  另加 AppParams（原 params.hpp）。
//
//  【设计原则】
//    · 本文件是 app 的**叶子头**：不 include 任何 app 文件；
//    · 只定义“东西长什么样”，不含逻辑；
//    · 具体数据源/出口的实现（Mavlink/Ros2/SimTruth）只在这里**声明**，
//      实现落地前不要实例化；真机胶水以后放 app/glue/，不把 driver 拖进来。
// =============================================================================

#include <cstdint>

#include "guidance_types.hpp"   // PureGuidanceMode / GuidanceParams
#include "vec3.hpp"             // pure_uav::Vec3

namespace pure_uav {
namespace app {

// ============================================================================
//  §1  采样
// ============================================================================

// 飞控惯导一帧（对应通信模块的 LocalPosition + Attitude）
struct InsSample {
    std::uint64_t boot_us = 0;                 // 采样时刻（飞控 boot 时钟，µs）
    Vec3   p;                                  // 位置（m，外部 NED 口径）
    Vec3   v;                                  // 速度（m/s，外部 NED 口径）
    double q[4] = {1.0, 0.0, 0.0, 0.0};        // 姿态 [w,x,y,z]（机体→世界）
    bool   has_position = false;
    bool   has_attitude = false;
};

// 检测/视觉一帧（目标方向以像素给出；像素→视线由主循环完成）
struct TargetSample {
    std::uint64_t boot_us = 0;                 // 曝光时刻（飞控 boot 时钟，µs）
    double u_px = 0.0;                         // 检测框中心 x（像素）
    double v_px = 0.0;                         // 检测框中心 y（像素）
    bool   has_box = false;                    // 是否带框
    double w_px = 0.0;                         // 框宽（像素）
    double h_px = 0.0;                         // 框高（像素）
    double conf = 0.0;                         // 置信度 0~1
    int    class_id = -1;                      // 类别（-1 = 未给）
    bool   present = false;                    // 本帧是否真的看到目标
    bool   has_range = false;                  // 是否测得距离
    double range_m = 0.0;                      // 距离（m）
    bool   has_v_T = false;                    // 目标速度（仅仿真真值有）
    Vec3   v_T;                                // 目标速度（外部 NED）
};

// ============================================================================
//  §2  输出与状态
// ============================================================================

enum class InputHealth : std::uint8_t {
    kOk = 0,
    kNotFinite,
    kLosZero,
    kLosNotUnit,
    kQuatNotUnit,
    kRangeNegative,
    kStale,
    kTimeBackwards,
};

enum class ControlMode : std::uint8_t {
    kAcceleration = 0,   // 发加速度（a_cmd_ned 有效）
    kVelocity     = 1,   // 发速度（v_cmd_ned 有效；零速度 = 悬停）
};

struct GuidanceOutput {
    std::uint64_t boot_us = 0;                 // 对应采样时刻（飞控 boot µs）
    ControlMode   ctrl = ControlMode::kAcceleration;
    Vec3   a_cmd_ned;                          // 期望加速度（NED，m/s²）
    Vec3   v_cmd_ned;                          // 期望速度（NED，m/s）
    bool   hold_yaw = true;                    // 意图标记（通信侧已忽略 yaw）
    double a_cmd_norm = 0.0;                   // 限幅后模长
    bool   sat = false;                        // 是否撞 a_max
    PureGuidanceMode mode = PureGuidanceMode::SEARCH;  // 导引律层面
    bool   valid = false;                      // 本拍输入是否可用于制导
    double t_go = 0.0;
    double Vc = 0.0;
    double R = 0.0;
    double lam_dot = 0.0;
    InputHealth health = InputHealth::kOk;
};

enum class LoopStatus : std::uint8_t {
    kOk = 0,
    kNotInitialized,
    kStopped,
    kSensorTimeout,
    kUnhealthyInput,
    kSinkError,
    kEstimatorReset,      // 预留：估计器重置监控
};

// ============================================================================
//  §3  端口（拉模型）
// ============================================================================

class IInsSource {
public:
    virtual ~IInsSource() = default;
    virtual bool poll(InsSample &out) = 0;
    virtual bool alive() const = 0;
};

class ITargetSource {
public:
    virtual ~ITargetSource() = default;
    virtual bool poll(TargetSample &out) = 0;
    virtual bool alive() const = 0;
};

class ICommandSink {
public:
    virtual ~ICommandSink() = default;
    virtual bool send(const GuidanceOutput &out) = 0;
    virtual bool alive() const = 0;
};

// 空出口：调试“只看算得对不对、不发指令”时用。
// 直接 inline，避免 app_core 反向依赖 tools。
class NullSink final : public ICommandSink {
public:
    bool send(const GuidanceOutput &) override { return true; }
    bool alive() const override { return true; }
};

// ============================================================================
//  §4  配置
// ============================================================================

struct CameraModel {
    double fx = 0.0, fy = 0.0;
    double cx = 0.0, cy = 0.0;
    // 外参：相机系 → 机体系（行主序 9 个 double）
    double R_body_cam[9] = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};
    bool valid() const { return fx > 0.0 && fy > 0.0; }
};

struct LoopConfig {
    int loop_hz = 50;
    double max_sample_age_s = 0.10;
    double sensor_timeout_s = 0.50;

    enum class OnUnhealthy : std::uint8_t {
        kHoldLastCommand = 0,   // 保持上一拍（限时）
        kSafeHover = 1,         // 零速度悬停
        kStop = 2,              // 不发，交给飞控 failsafe
    };
    OnUnhealthy on_unhealthy = OnUnhealthy::kHoldLastCommand;

    double hold_last_max_s = 0.20;
    double full_check_hz = 2.0;
    bool   send_on_every_tick = true;
    CameraModel cam;
};

// 整机启动配置（机载只认这一份）
struct AppParams {
    GuidanceParams guidance;
    LoopConfig     loop;
};

// ============================================================================
//  §5  预留实现声明（只有声明，没有实现）
// ============================================================================
//
// 真机/仿真实现落地前不要实例化。具体实现文件才 include 通信/检测头文件；
// 本头保持零外部依赖。落地时在 app/glue/ 实现，再由 main 链接 communicator。
//
//  class SimTruthInsSource   final : public IInsSource  { ... };
//  class MavlinkInsSource     final : public IInsSource  { ... };
//  class SimTruthTargetSource final : public ITargetSource { ... };
//  class VisionTargetSource   final : public ITargetSource { ... };
//  class MavlinkAccelSink     final : public ICommandSink { ... };
//  class Ros2AccelSink        final : public ICommandSink { ... };

}  // namespace app
}  // namespace pure_uav
