#pragma once

#include <cstdint>
#include <optional>

#include "guidance_flight_types.hpp"
#include "msg.hpp"

namespace pure_uav::app::mission {

// 任务层只决定阶段和控制权，制导律仍由 guidance 模块负责。
enum class MissionState : std::uint8_t {
    kIdle,
    kPreflight,
    kOffboard,
    kArming,
    kTakeoff,
    kSearch,
    kStrike,
    kReturnHome,
    kLanding,
    kCompleted,
    kAborted,
};

enum class MissionReason : std::uint8_t {
    kNone,
    kPreflightTimeout,
    kStageTimeout,
    kSearchTimeout,
    kStrikeTimeout,
    kMissionTimeout,
    kOperatorReturn,
    kManualTakeover,
    kLinkLost,
    kPoseInvalid,
    kOffboardLost,
    kUnexpectedDisarm,
    kCommandFailed,
    kInvalidTime,
    kSessionChanged,
    kNavigationReset,
};

// 边沿请求，由宿主异步执行。不要在 tick/VIO 发布线程等待 COMMAND_ACK。
enum class FlightRequest : std::uint8_t { kNone, kOffboard, kArm, kLand };

struct MissionConfig {
    double takeoff_height_m = 1.5;  // 相对起飞点的正高度；NED 目标 z = home.z - height。
    double position_tolerance_m = 0.20;
    double settled_speed_mps = 0.20;
    double settle_time_s = 0.50;
    double pose_max_age_s = 0.20;
    double target_max_age_s = 0.20;
    double target_lost_timeout_s = 0.50;
    double min_target_confidence = 0.60;
    int target_class = -1;  // -1 接受全部类别；与检测配置约定保持一致。
    unsigned target_confirm_frames = 3;
    double preflight_timeout_s = 10.0;
    double setup_timeout_s = 10.0;
    double takeoff_timeout_s = 15.0;
    double search_timeout_s = 30.0;
    double strike_timeout_s = 5.0;
    double return_timeout_s = 20.0;
    double landing_timeout_s = 20.0;
    double mission_timeout_s = 120.0;
    double request_retry_s = 1.0;
};

struct MissionInput {
    // 全部 *_time_s 与 now_s 使用同一个主机 steady_clock 原点，单位秒。
    // 时间戳必须来自采样时刻映射，不能每次读缓存时重新盖章。
    double now_s = 0.0;
    bool start_requested = false;  // 仅 Idle 接受；终态需显式 reset 后重启。
    bool return_requested = false;
    bool manual_takeover = false;  // 立即释放控制权。
    bool command_failed = false;   // 宿主报告发送失败或明确的命令拒绝。
    bool fcu_connected = false;    // 宿主须检查心跳新鲜度。
    bool armed = false;
    bool offboard = false;
    bool landed = false;                 // 来自飞控落地状态，不能用高度接近零代替。
    bool preflight_ready = false;        // 定位/电量/人工启动条件等整机自检结果。
    bool offboard_stream_ready = false;  // 宿主确认实际成功发送的 setpoint 预热流达标。
    bool guidance_ready = false;         // 相机标定、测距、制导端口和配置均已就绪。
    bool strike_confirmed = false;       // 撞击传感器/裁判/仿真事件；距离小不等于撞击成功。

    guidance::flight::GuidanceInsSample ins;
    double pose_time_s = -1.0;
    guidance::flight::GuidanceTargetSample target;
    double target_time_s = -1.0;
};

struct MissionOutput {
    MissionState state = MissionState::kIdle;
    MissionReason reason = MissionReason::kNone;
    FlightRequest request = FlightRequest::kNone;
    std::optional<GuidanceCmd> setpoint;
    bool run_guidance = false;    // 本拍由 GuidanceLoop::run_once() 独占指令出口。
    bool reset_guidance = false;  // 进入 Strike 的首拍；先重置制导记忆，再运行。
    bool state_changed = false;
    bool strike_confirmed = false;
};

const char* toString(MissionState state);
const char* toString(MissionReason reason);

}  // namespace pure_uav::app::mission
