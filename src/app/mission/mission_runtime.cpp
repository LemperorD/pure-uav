#include "mission/mission_runtime.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace pure_uav::app::mission {
namespace {
bool terminal(MissionState state) {
    return state == MissionState::kCompleted || state == MissionState::kAborted;
}
bool finiteCommand(const GuidanceCmd& command) {
    switch (command.mode) {
        case GuidanceMode::POSITION:
            return std::isfinite(command.x) && std::isfinite(command.y) && std::isfinite(command.z);
        case GuidanceMode::VELOCITY:
            return std::isfinite(command.vx) && std::isfinite(command.vy) && std::isfinite(command.vz);
        case GuidanceMode::POS_WITH_VEL_FEEDFORWARD:
            return std::isfinite(command.x) && std::isfinite(command.y) && std::isfinite(command.z) && std::isfinite(command.vx) && std::isfinite(command.vy) && std::isfinite(command.vz);
        case GuidanceMode::ACCELERATION:
            return std::isfinite(command.afx) && std::isfinite(command.afy) && std::isfinite(command.afz);
    }
    return false;
}
}  // namespace

MissionRuntime::MissionRuntime(RuntimeConfig config, MissionFlightPort& flight, MissionGuidancePort* guidance) : config_(config), controller_(config.mission), flight_(flight), guidance_(guidance) {
    if (config.mission.pose_max_age_s > std::numeric_limits<int>::max() / 1000.0 || config.mission.target_max_age_s > 1e9 || !std::isfinite(config.prestream_s) || config.prestream_s <= 0.0 || !std::isfinite(config.max_setpoint_gap_s) || config.max_setpoint_gap_s <= 0.0 ||
        config.max_setpoint_gap_s >= config.prestream_s || config.prestream_s >= config.mission.setup_timeout_s) {
        throw std::invalid_argument("mission runtime: invalid prestream duration or send gap");
    }
}

void MissionRuntime::clearStream() {
    stream_started_s_ = last_sent_s_ = -1.0;
}

void MissionRuntime::fail(MissionReason reason, double now_s) {
    const bool strike_confirmed = status_.mission.strike_confirmed;
    controller_.cancel(reason, now_s);
    status_.mission = {};
    status_.mission.strike_confirmed = strike_confirmed;
    status_.mission.state = controller_.state();
    status_.mission.reason = controller_.reason();
    status_.mission.state_changed = true;
    clearStream();
}

const RuntimeStatus& MissionRuntime::tick(const RuntimeSignals& signals) {
    if (stopped_ || terminal(status_.mission.state)) {
        status_.mission.state_changed = false;
        return status_;
    }
    last_now_s_ = signals.now_s;
    const auto feedback = flight_.feedback(signals.now_s);
    MissionInput input = feedback.input;
    input.now_s = signals.now_s;
    input.start_requested = signals.start_requested;
    input.return_requested = signals.return_requested;
    input.manual_takeover = signals.manual_takeover;
    input.strike_confirmed = signals.strike_confirmed;
    input.target = signals.target;
    input.target_time_s = signals.target_time_s;
    input.guidance_ready = guidance_ != nullptr && guidance_->ready();
    input.preflight_ready = input.preflight_ready && feedback.supported && signals.navigation_ready;
    if (!signals.navigation_ready) {
        input.ins.has_position = false;
    }
    if (signals.return_requested && controller_.state() == MissionState::kIdle) {
        fail(MissionReason::kOperatorReturn, signals.now_s);
        return status_;
    }
    if (session_) {
        if (*session_ != feedback.session) {
            fail(MissionReason::kSessionChanged, signals.now_s);
            return status_;
        }
        if (navigation_generation_ != signals.navigation_generation) {
            fail(MissionReason::kNavigationReset, signals.now_s);
            return status_;
        }
    }
    if (last_sent_s_ < 0.0 || signals.now_s < last_sent_s_ || signals.now_s - last_sent_s_ > config_.max_setpoint_gap_s) {
        clearStream();
    }
    input.offboard_stream_ready = config_.send_enabled && stream_started_s_ >= 0.0 && signals.now_s - stream_started_s_ >= config_.prestream_s;
    status_.mission = controller_.tick(input);
    auto& output = status_.mission;
    if (!session_ && controller_.hasHome()) {
        session_ = feedback.session;
        navigation_generation_ = signals.navigation_generation;
    }
    if (terminal(output.state)) {
        clearStream();
        return status_;
    }
    if (output.reset_guidance && (guidance_ == nullptr || !guidance_->reset())) {
        fail(MissionReason::kCommandFailed, signals.now_s);
        return status_;
    }
    std::optional<GuidanceCmd> command = output.setpoint;
    if (output.run_guidance) {
        command = guidance_ == nullptr ? std::nullopt : guidance_->step(input);
        if (!command) {
            fail(MissionReason::kCommandFailed, signals.now_s);
            return status_;
        }
    }
    if (command) {
        if (!finiteCommand(*command)) {
            fail(MissionReason::kCommandFailed, signals.now_s);
            return status_;
        }
        if (!config_.send_enabled) {
            ++status_.would_send;
        } else if (!flight_.send(*command, feedback.session)) {
            fail(MissionReason::kCommandFailed, signals.now_s);
            return status_;
        } else {
            ++status_.setpoints_sent;
            if (stream_started_s_ < 0.0) {
                stream_started_s_ = signals.now_s;
            }
            last_sent_s_ = signals.now_s;
        }
    } else {
        clearStream();
    }
    // 先发 setpoint，再发请求。失败、取消或终态均不会进入此分支。
    if (output.request != FlightRequest::kNone) {
        if (!config_.send_enabled) {
            ++status_.would_send;
        } else if (!flight_.request(output.request, feedback.session)) {
            fail(MissionReason::kCommandFailed, signals.now_s);
        } else {
            ++status_.requests_sent;
        }
    }
    return status_;
}

void MissionRuntime::stop() {
    if (!stopped_ && !terminal(status_.mission.state)) {
        fail(MissionReason::kManualTakeover, last_now_s_);
    }
    stopped_ = true;
}

const RuntimeStatus& MissionRuntime::status() const {
    return status_;
}

}  // namespace pure_uav::app::mission
