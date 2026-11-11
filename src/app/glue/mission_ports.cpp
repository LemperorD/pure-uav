#include "glue/mission_ports.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>

#include "runtime/guidance_ports.hpp"

namespace pure_uav::app {
namespace {
int poseMaxAgeMs(double age_s) {
    if (!std::isfinite(age_s) || age_s <= 0 || age_s > std::numeric_limits<int>::max() / 1000.0) {
        throw std::invalid_argument("mission pose age cannot be represented in milliseconds");
    }
    return static_cast<int>(std::ceil(age_s * 1000.0));
}
}  // namespace

MavlinkMissionPort::MavlinkMissionPort(Communicator& comm, double pose_max_age_s, int heartbeat_max_age_ms, PreflightConfig preflight) : comm_(comm), pose_max_age_ms_(poseMaxAgeMs(pose_max_age_s)), heartbeat_max_age_ms_(heartbeat_max_age_ms), preflight_(preflight) {
}

mission::FlightFeedback MavlinkMissionPort::feedback(double now_s) {
    mission::FlightFeedback result;
    result.session = comm_.reconnectCount();
    result.supported = comm_.getAutopilot() == autopilot::PX4;
    const bool session_stable = has_preflight_session_ && preflight_session_ == result.session;
    if (!has_preflight_session_ || preflight_session_ != result.session) {
        preflight_session_ = result.session;
        has_preflight_session_ = true;
    }
    if (session_ != result.session) {
        session_ = result.session;
        pose_boot_us_ = 0;
        pose_time_s_ = -1.0;
    }
    auto& input = result.input;
    input.now_s = now_s;
    const auto heartbeat_age = comm_.lastHeartbeatAgeMs();
    input.fcu_connected = comm_.isConnected() && heartbeat_age >= 0 && heartbeat_age <= heartbeat_max_age_ms_;
    input.armed = comm_.isArmed();
    input.offboard = comm_.getCustomMode() == px4_mode::OFFBOARD;
    LandedState landed;
    const bool have_landed = comm_.getLandedState(landed);
    input.landed = have_landed && landed.state == 1;

    Attitude attitude;
    LocalPosition position;
    int64_t attitude_ns = 0;
    int64_t position_ns = 0;
    const int attitude_limit_ms = std::max(pose_max_age_ms_, preflight_.config().attitude_max_age_ms);
    const int position_limit_ms = std::max(pose_max_age_ms_, preflight_.config().local_position_max_age_ms);
    const bool attitude_ok = comm_.getAttitude(attitude, attitude_limit_ms, &attitude_ns);
    const bool position_ok = comm_.getLocalPosition(position, position_limit_ms, &position_ns);
    if (attitude_ok && position_ok && makeGuidanceIns(attitude, position, 0.05, input.ins)) {
        if (input.ins.boot_us > pose_boot_us_) {
            pose_boot_us_ = input.ins.boot_us;
            pose_time_s_ = static_cast<double>(std::min(attitude_ns, position_ns)) * 1e-9;
        } else if (input.ins.boot_us < pose_boot_us_) {
            // 不能沿用飞控重启前的本地返航点。
            input.ins.has_position = false;
        }
        input.pose_time_s = pose_time_s_;  // 重读缓存或重复 boot 时间戳不刷新年龄。
    }

    SystemTime system_time;
    const bool system_time_ok = comm_.getSystemTime(system_time, preflight_.config().system_time_max_age_ms);
    Battery battery;
    const bool battery_seen = comm_.getCountBattery() > 0 && comm_.getBattery(battery);

    PreflightInput preflight_input;
    preflight_input.connected = input.fcu_connected;
    preflight_input.target_locked = comm_.targetLocked();
    preflight_input.autopilot_px4 = result.supported;
    preflight_input.session_stable = session_stable;
    preflight_input.armed = input.armed;
    preflight_input.landed = input.landed;
    preflight_input.mode = comm_.getCustomMode();
    preflight_input.heartbeat_age_ms = heartbeat_age;
    preflight_input.attitude_seen = attitude_ok;
    preflight_input.attitude_age_ms = comm_.attitudeAgeMs();
    preflight_input.attitude = attitude;
    preflight_input.position_seen = position_ok;
    preflight_input.position_age_ms = comm_.localPositionAgeMs();
    preflight_input.position = position;
    preflight_input.battery_seen = battery_seen;
    preflight_input.battery = battery;
    preflight_input.system_time_seen = system_time_ok;
    preflight_input.system_time_age_ms = comm_.systemTimeAgeMs();

    const PreflightResult preflight = preflight_.evaluate(preflight_input);
    input.preflight_ready = preflight.ok;
    if (!preflight.ok && preflight.reason != last_preflight_reason_) {
        std::fprintf(stderr, "[PREFLIGHT] %s\n", preflight.reason.c_str());
        last_preflight_reason_ = preflight.reason;
    } else if (preflight.ok && !last_preflight_reason_.empty()) {
        std::fprintf(stderr, "[PREFLIGHT] ready\n");
        last_preflight_reason_.clear();
    }

    if (comm_.reconnectCount() != result.session) {
        input.fcu_connected = input.preflight_ready = false;
        input.ins = {};
    }
    return result;
}

bool MavlinkMissionPort::send(const GuidanceCmd& command, std::uint64_t session) {
    return comm_.pushGuidanceCmdForSession(command, session);
}

bool MavlinkMissionPort::request(mission::FlightRequest request, std::uint64_t session) {
    switch (request) {
        case mission::FlightRequest::kOffboard:
            return comm_.requestOffboardForSession(session);
        case mission::FlightRequest::kArm:
            return comm_.requestArmForSession(session);
        case mission::FlightRequest::kLand:
            return comm_.requestLandForSession(session);
        case mission::FlightRequest::kNone:
            return true;
    }
    return false;
}

MissionGuidanceRunner::MissionGuidanceRunner(GuidanceAppParams params) : params_(std::move(params)) {
    reset();
}

bool MissionGuidanceRunner::ready() const {
    return loop_ && loop_->initialized();
}

bool MissionGuidanceRunner::reset() {
    loop_.reset();
    ins_.value = {};
    ins_.delivered_us = 0;
    target_.value = {};
    target_.delivered_us = 0;
    loop_.emplace(params_.guidance, params_.loop, ins_, target_, sink_);
    if (!loop_->init()) {
        loop_.reset();
        return false;
    }
    return true;
}

std::optional<GuidanceCmd> MissionGuidanceRunner::step(const mission::MissionInput& input) {
    if (!ready()) {
        return std::nullopt;
    }
    ins_.value = input.ins;
    target_.value = input.target;
    flight::GuidanceOutput output;
    const auto status = loop_->run_once(&output);
    if (status != flight::GuidanceLoopStatus::kOk || !output.valid || output.health != flight::GuidanceInputHealth::kOk) {
        return std::nullopt;
    }
    return makeGuidanceCmd(output);
}

bool MissionGuidanceRunner::InsSource::poll(flight::GuidanceInsSample& out) {
    if (value.boot_us == 0 || value.boot_us <= delivered_us) {
        return false;
    }
    delivered_us = value.boot_us;
    out = value;
    return true;
}
bool MissionGuidanceRunner::InsSource::alive() const {
    return true;
}
bool MissionGuidanceRunner::TargetSource::poll(flight::GuidanceTargetSample& out) {
    if (value.boot_us == 0 || value.boot_us <= delivered_us) {
        return false;
    }
    delivered_us = value.boot_us;
    out = value;
    return true;
}
bool MissionGuidanceRunner::TargetSource::alive() const {
    return true;
}

}  // namespace pure_uav::app
