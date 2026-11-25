#include "mission/mission_controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace pure_uav::app::mission {
namespace {

bool finite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool fresh(double now_s, double sample_s, double max_age_s) {
    return std::isfinite(sample_s) && sample_s >= 0.0 && sample_s <= now_s && now_s - sample_s <= max_age_s;
}

bool terminal(MissionState state) {
    return state == MissionState::kCompleted || state == MissionState::kAborted;
}

bool airborneStage(MissionState state) {
    return state == MissionState::kTakeoff || state == MissionState::kSearch || state == MissionState::kStrike || state == MissionState::kReturnHome;
}

GuidanceCmd positionCommand(const Vec3& position) {
    GuidanceCmd command;
    command.mode = GuidanceMode::POSITION;
    command.x = static_cast<float>(position.x);
    command.y = static_cast<float>(position.y);
    command.z = static_cast<float>(position.z);
    return command;
}

}  // namespace

MissionController::MissionController(MissionConfig config) : config_(config) {
    const double positive[] = {config.takeoff_height_m, config.position_tolerance_m, config.settled_speed_mps, config.settle_time_s,    config.pose_max_age_s,   config.target_max_age_s,  config.target_lost_timeout_s, config.preflight_timeout_s,
                               config.setup_timeout_s,  config.takeoff_timeout_s,    config.search_timeout_s,  config.strike_timeout_s, config.return_timeout_s, config.landing_timeout_s, config.mission_timeout_s,     config.request_retry_s};
    for (double value : positive) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("mission: distances, speeds and durations must be finite and positive");
        }
    }
    if (!std::isfinite(config.min_target_confidence) || config.min_target_confidence < 0.0 || config.min_target_confidence > 1.0 || config.target_class < -1 || config.target_confirm_frames == 0 || config.takeoff_height_m <= config.position_tolerance_m ||
        config.takeoff_height_m > std::numeric_limits<float>::max() / 4.0) {
        throw std::invalid_argument("mission: invalid target selection or takeoff tolerance");
    }
}

void MissionController::reset() {
    *this = MissionController(config_);
}

void MissionController::cancel(MissionReason reason, double now_s) {
    if (!terminal(state_)) {
        abort(reason, now_s);
    }
}

void MissionController::enter(MissionState state, double now_s) {
    state_ = state;
    entered_s_ = now_s;
    settled_since_s_ = -1.0;
    last_request_s_ = -1.0;
    confirmed_frames_ = 0;
    holding_after_loss_ = false;
    if (state == MissionState::kSearch) {
        last_target_boot_us_ = 0;
        last_target_s_ = -1.0;
    }
}

void MissionController::abort(MissionReason reason, double now_s) {
    reason_ = reason;
    enter(MissionState::kAborted, now_s);
}

void MissionController::returnHome(MissionReason reason, const MissionInput& input) {
    reason_ = reason;
    // 起飞尚未离地就取消时，不再发送升高到返航点的指令。
    enter(input.landed ? MissionState::kLanding : MissionState::kReturnHome, input.now_s);
}

bool MissionController::poseUsable(const MissionInput& input) const {
    if (!input.ins.has_position || !input.ins.has_attitude || !fresh(input.now_s, input.pose_time_s, config_.pose_max_age_s) || !finite(input.ins.p) || !finite(input.ins.v)) {
        return false;
    }
    // 下游 GuidanceCmd 使用 float，给相对起飞高度的加减也留出表示范围。
    constexpr double limit = std::numeric_limits<float>::max() / 4.0;
    if (std::abs(input.ins.p.x) > limit || std::abs(input.ins.p.y) > limit || std::abs(input.ins.p.z) > limit) {
        return false;
    }
    double q_norm2 = 0.0;
    for (double q : input.ins.q) {
        q_norm2 += q * q;
    }
    return std::isfinite(q_norm2) && std::abs(q_norm2 - 1.0) <= 0.05;
}

bool MissionController::targetUsable(const MissionInput& input) const {
    const auto& target = input.target;
    return fresh(input.now_s, input.target_time_s, config_.target_max_age_s) && target.boot_us > 0 && target.present && std::isfinite(target.conf) && target.conf >= config_.min_target_confidence && target.conf <= 1.0 && (config_.target_class < 0 || target.class_id == config_.target_class) &&
           std::isfinite(target.u_px) && std::isfinite(target.v_px) && target.has_range && std::isfinite(target.range_m) && target.range_m > 0.0;
}

bool MissionController::settled(const MissionInput& input, const Vec3& destination) {
    if (norm(input.ins.p - destination) > config_.position_tolerance_m || norm(input.ins.v) > config_.settled_speed_mps) {
        settled_since_s_ = -1.0;
        return false;
    }
    if (settled_since_s_ < 0.0) {
        settled_since_s_ = input.now_s;
    }
    return input.now_s - settled_since_s_ >= config_.settle_time_s;
}

bool MissionController::requestDue(double now_s) {
    if (last_request_s_ >= 0.0 && now_s - last_request_s_ < config_.request_retry_s) {
        return false;
    }
    last_request_s_ = now_s;
    return true;
}

MissionOutput MissionController::tick(const MissionInput& input) {
    const auto previous = state_;
    if (terminal(state_)) {
        return output(input, previous);
    }
    if (!std::isfinite(input.now_s) || input.now_s < 0.0 || input.now_s < last_tick_s_) {
        abort(MissionReason::kInvalidTime, last_tick_s_);
        return output(input, previous);
    }
    const double tick_gap_s = last_tick_s_ < 0.0 ? 0.0 : input.now_s - last_tick_s_;
    last_tick_s_ = input.now_s;
    if (tick_gap_s > config_.pose_max_age_s) {
        // 长时间没有调度不能算作持续稳定或连续看到目标。
        settled_since_s_ = -1.0;
        confirmed_frames_ = 0;
    }
    if (state_ == MissionState::kIdle) {
        if (input.start_requested && !input.manual_takeover && !input.return_requested) {
            started_s_ = input.now_s;
            enter(MissionState::kPreflight, input.now_s);
        }
        return output(input, previous);
    }
    if (input.manual_takeover) {
        abort(MissionReason::kManualTakeover, input.now_s);
        return output(input, previous);
    }
    // Preflight 等待数据就绪；其后丢失链路/定位则释放控制权，交飞控处理。
    if (state_ != MissionState::kPreflight) {
        if (!input.fcu_connected) {
            abort(MissionReason::kLinkLost, input.now_s);
        } else if (input.command_failed) {
            abort(MissionReason::kCommandFailed, input.now_s);
        } else if (state_ != MissionState::kLanding && !poseUsable(input)) {
            abort(MissionReason::kPoseInvalid, input.now_s);
        } else if ((airborneStage(state_) && !input.armed) || (state_ == MissionState::kOffboard && input.armed)) {
            abort(MissionReason::kUnexpectedDisarm, input.now_s);
        } else if ((airborneStage(state_) || state_ == MissionState::kArming) && !input.offboard) {
            abort(MissionReason::kOffboardLost, input.now_s);
        }
        if (terminal(state_)) {
            return output(input, previous);
        }
    }
    if (input.return_requested && state_ != MissionState::kReturnHome && state_ != MissionState::kLanding) {
        if (airborneStage(state_)) {
            returnHome(MissionReason::kOperatorReturn, input);
        } else {
            abort(MissionReason::kOperatorReturn, input.now_s);
        }
        return output(input, previous);
    }
    if (input.now_s - started_s_ >= config_.mission_timeout_s && state_ != MissionState::kReturnHome && state_ != MissionState::kLanding) {
        if (airborneStage(state_)) {
            returnHome(MissionReason::kMissionTimeout, input);
        } else {
            abort(MissionReason::kMissionTimeout, input.now_s);
        }
        return output(input, previous);
    }

    const double elapsed_s = input.now_s - entered_s_;
    switch (state_) {
        case MissionState::kPreflight:
            if (input.fcu_connected && input.preflight_ready && !input.command_failed && !input.armed && input.landed && poseUsable(input) && norm(input.ins.v) <= config_.settled_speed_mps) {
                home_ned_ = input.ins.p;
                takeoff_ned_ = home_ned_;
                takeoff_ned_.z -= config_.takeoff_height_m;
                search_ned_ = takeoff_ned_;
                have_home_ = true;
                enter(MissionState::kOffboard, input.now_s);
            } else if (elapsed_s >= config_.preflight_timeout_s) {
                abort(MissionReason::kPreflightTimeout, input.now_s);
            }
            break;
        case MissionState::kOffboard:
            if (input.offboard && input.offboard_stream_ready) {
                enter(MissionState::kArming, input.now_s);
            } else if (elapsed_s >= config_.setup_timeout_s) {
                abort(MissionReason::kStageTimeout, input.now_s);
            }
            break;
        case MissionState::kArming:
            if (input.armed) {
                enter(MissionState::kTakeoff, input.now_s);
            } else if (elapsed_s >= config_.setup_timeout_s) {
                abort(MissionReason::kStageTimeout, input.now_s);
            }
            break;
        case MissionState::kTakeoff:
            if (!input.landed && settled(input, takeoff_ned_)) {
                enter(MissionState::kSearch, input.now_s);
            } else if (elapsed_s >= config_.takeoff_timeout_s) {
                returnHome(MissionReason::kStageTimeout, input);
            }
            break;
        case MissionState::kSearch:
            if (!targetUsable(input) || !input.guidance_ready) {
                confirmed_frames_ = 0;
            } else if (input.target.boot_us > last_target_boot_us_ && input.target_time_s >= last_target_s_) {
                if (last_target_s_ >= 0.0 && input.target_time_s - last_target_s_ > config_.target_max_age_s) {
                    confirmed_frames_ = 0;
                }
                last_target_boot_us_ = input.target.boot_us;
                last_target_s_ = input.target_time_s;
                ++confirmed_frames_;
            } else if (input.target.boot_us < last_target_boot_us_ || input.target_time_s < last_target_s_) {
                // 检测时钟回退不能沿用上一段确认计数。
                confirmed_frames_ = 0;
            }
            if (confirmed_frames_ >= config_.target_confirm_frames) {
                enter(MissionState::kStrike, input.now_s);
            } else if (elapsed_s >= config_.search_timeout_s) {
                returnHome(MissionReason::kSearchTimeout, input);
            }
            break;
        case MissionState::kStrike:
            if (input.strike_confirmed) {
                strike_confirmed_ = true;
                returnHome(MissionReason::kNone, input);
            } else if (elapsed_s >= config_.strike_timeout_s) {
                returnHome(MissionReason::kStrikeTimeout, input);
            } else {
                if (targetUsable(input) && input.guidance_ready && input.target.boot_us > last_target_boot_us_ && input.target_time_s >= last_target_s_) {
                    last_target_boot_us_ = input.target.boot_us;
                    last_target_s_ = input.target_time_s;
                }
                if (input.now_s - last_target_s_ >= config_.target_lost_timeout_s) {
                    search_ned_ = input.ins.p;
                    enter(MissionState::kSearch, input.now_s);
                }
            }
            break;
        case MissionState::kReturnHome:
            if (settled(input, takeoff_ned_)) {
                enter(MissionState::kLanding, input.now_s);
            } else if (elapsed_s >= config_.return_timeout_s) {
                // 不能一直飞向无法到达的返航点：切飞控自主降落。
                reason_ = MissionReason::kStageTimeout;
                enter(MissionState::kLanding, input.now_s);
            }
            break;
        case MissionState::kLanding:
            if (input.landed && !input.armed) {
                enter(MissionState::kCompleted, input.now_s);
            } else if (elapsed_s >= config_.landing_timeout_s) {
                abort(MissionReason::kStageTimeout, input.now_s);
            }
            break;
        case MissionState::kIdle:
        case MissionState::kCompleted:
        case MissionState::kAborted:
            break;
    }
    return output(input, previous);
}

MissionOutput MissionController::output(const MissionInput& input, MissionState previous) {
    MissionOutput result;
    result.state = state_;
    result.reason = reason_;
    result.state_changed = state_ != previous;
    result.strike_confirmed = strike_confirmed_;
    switch (state_) {
        case MissionState::kOffboard:
            result.setpoint = positionCommand(home_ned_);
            if (input.offboard_stream_ready && !input.offboard && requestDue(input.now_s)) {
                result.request = FlightRequest::kOffboard;
            }
            break;
        case MissionState::kArming:
            result.setpoint = positionCommand(home_ned_);
            if (!input.armed && requestDue(input.now_s)) {
                result.request = FlightRequest::kArm;
            }
            break;
        case MissionState::kTakeoff:
        case MissionState::kReturnHome:
            result.setpoint = positionCommand(takeoff_ned_);
            break;
        case MissionState::kSearch:
            result.setpoint = positionCommand(search_ned_);
            break;
        case MissionState::kStrike:
            result.reset_guidance = result.state_changed;
            if (targetUsable(input) && input.guidance_ready && input.target.boot_us >= last_target_boot_us_ && input.target_time_s >= last_target_s_) {
                result.run_guidance = true;
                holding_after_loss_ = false;
            } else {
                if (!holding_after_loss_) {
                    loss_hold_ned_ = input.ins.p;
                    holding_after_loss_ = true;
                }
                result.setpoint = positionCommand(loss_hold_ned_);
            }
            break;
        case MissionState::kLanding:
            // 降落阶段不再流式发送 Offboard setpoint，控制权交飞控。
            if (input.armed && requestDue(input.now_s)) {
                result.request = FlightRequest::kLand;
            }
            break;
        case MissionState::kIdle:
        case MissionState::kPreflight:
        case MissionState::kCompleted:
        case MissionState::kAborted:
            break;
    }
    return result;
}

const char* toString(MissionState state) {
    switch (state) {
        case MissionState::kIdle:
            return "idle";
        case MissionState::kPreflight:
            return "preflight";
        case MissionState::kOffboard:
            return "offboard";
        case MissionState::kArming:
            return "arming";
        case MissionState::kTakeoff:
            return "takeoff";
        case MissionState::kSearch:
            return "search";
        case MissionState::kStrike:
            return "strike";
        case MissionState::kReturnHome:
            return "return_home";
        case MissionState::kLanding:
            return "landing";
        case MissionState::kCompleted:
            return "completed";
        case MissionState::kAborted:
            return "aborted";
    }
    return "unknown";
}

const char* toString(MissionReason reason) {
    switch (reason) {
        case MissionReason::kNone:
            return "none";
        case MissionReason::kPreflightTimeout:
            return "preflight_timeout";
        case MissionReason::kStageTimeout:
            return "stage_timeout";
        case MissionReason::kSearchTimeout:
            return "search_timeout";
        case MissionReason::kStrikeTimeout:
            return "strike_timeout";
        case MissionReason::kMissionTimeout:
            return "mission_timeout";
        case MissionReason::kOperatorReturn:
            return "operator_return";
        case MissionReason::kManualTakeover:
            return "manual_takeover";
        case MissionReason::kLinkLost:
            return "link_lost";
        case MissionReason::kPoseInvalid:
            return "pose_invalid";
        case MissionReason::kOffboardLost:
            return "offboard_lost";
        case MissionReason::kUnexpectedDisarm:
            return "unexpected_disarm";
        case MissionReason::kCommandFailed:
            return "command_failed";
        case MissionReason::kSessionChanged:
            return "session_changed";
        case MissionReason::kNavigationReset:
            return "navigation_reset";
        case MissionReason::kInvalidTime:
            return "invalid_time";
    }
    return "unknown";
}

}  // namespace pure_uav::app::mission
