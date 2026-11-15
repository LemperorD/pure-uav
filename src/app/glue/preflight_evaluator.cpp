#include "glue/preflight_evaluator.hpp"

#include <cmath>
#include <utility>

namespace pure_uav::app {
namespace {
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kQuaternionNormTolerance = 1e-3;

PreflightResult fail(std::string reason) {
    return PreflightResult{false, std::move(reason)};
}

bool fresh(std::int64_t age_ms, int max_age_ms) {
    return age_ms >= 0 && age_ms <= max_age_ms;
}

bool modeSafe(std::uint32_t mode) {
    return mode != 0 && mode != px4_mode::OFFBOARD && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LAND && mode != px4_mode::AUTO_MISSION && mode != px4_mode::AUTO_RTL;
}
}  // namespace

PreflightEvaluator::PreflightEvaluator(PreflightConfig config) : config_(config) {
}

PreflightResult PreflightEvaluator::evaluate(const PreflightInput& input) const {
    if (!input.connected) {
        return fail("飞控未连接");
    }
    if (!input.target_locked) {
        return fail("尚未锁定飞控目标");
    }
    if (!input.autopilot_px4) {
        return fail("飞控不是 PX4");
    }
    if (!input.session_stable) {
        return fail("飞控会话刚变化");
    }
    if (!fresh(input.heartbeat_age_ms, config_.heartbeat_max_age_ms)) {
        return fail("心跳不新鲜: " + std::to_string(input.heartbeat_age_ms) + " ms");
    }
    if (input.armed) {
        return fail("已解锁");
    }
    if (!input.landed) {
        return fail("飞控未确认在地面");
    }
    if (!modeSafe(input.mode)) {
        return fail("当前模式不允许起飞前自检");
    }
    if (!input.attitude_seen || !fresh(input.attitude_age_ms, config_.attitude_max_age_ms)) {
        return fail("姿态不新鲜: " + std::to_string(input.attitude_age_ms) + " ms");
    }
    if (!std::isfinite(input.attitude.roll) || !std::isfinite(input.attitude.pitch) || !std::isfinite(input.attitude.q[0]) || !std::isfinite(input.attitude.q[1]) || !std::isfinite(input.attitude.q[2]) || !std::isfinite(input.attitude.q[3])) {
        return fail("姿态含 NaN/Inf");
    }
    const double q_norm = std::sqrt(static_cast<double>(input.attitude.q[0]) * input.attitude.q[0] + static_cast<double>(input.attitude.q[1]) * input.attitude.q[1] + static_cast<double>(input.attitude.q[2]) * input.attitude.q[2] + static_cast<double>(input.attitude.q[3]) * input.attitude.q[3]);
    if (std::fabs(q_norm - 1.0) > kQuaternionNormTolerance) {
        return fail("姿态四元数模长异常");
    }
    if (std::fabs(static_cast<double>(input.attitude.roll) * kRadToDeg) > config_.max_tilt_deg || std::fabs(static_cast<double>(input.attitude.pitch) * kRadToDeg) > config_.max_tilt_deg) {
        return fail("姿态倾斜角过大");
    }
    if (!input.position_seen || !fresh(input.position_age_ms, config_.local_position_max_age_ms)) {
        return fail("局部位置不新鲜: " + std::to_string(input.position_age_ms) + " ms");
    }
    if (!std::isfinite(input.position.x) || !std::isfinite(input.position.y) || !std::isfinite(input.position.z) || !std::isfinite(input.position.vx) || !std::isfinite(input.position.vy) || !std::isfinite(input.position.vz)) {
        return fail("局部位置含 NaN/Inf");
    }
    if (std::fabs(static_cast<double>(input.position.vz)) > config_.max_vertical_speed_mps) {
        return fail("垂直速度过大");
    }
    if (!input.system_time_seen || !fresh(input.system_time_age_ms, config_.system_time_max_age_ms)) {
        return fail("SYSTEM_TIME 不新鲜: " + std::to_string(input.system_time_age_ms) + " ms");
    }
    if (!input.battery_seen) {
        return fail("未收到电池状态");
    }
    if (!std::isfinite(input.battery.voltage) || input.battery.voltage <= 0.0F) {
        return fail("电池电压无效");
    }
    if (input.battery.remaining < config_.min_battery_percent) {
        return fail("电量不足: " + std::to_string(static_cast<int>(input.battery.remaining)) + "%");
    }
    return PreflightResult{true, {}};
}

}  // namespace pure_uav::app
