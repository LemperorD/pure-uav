#include "startup/flight_safety.hpp"

#include <cmath>
#include <exception>
#include <optional>

#include "common/mavlink.h"
#include "communicator.hpp"
#include "startup/flight_flow.hpp"
#include "startup/logging.hpp"
#include "startup/main_config.hpp"

namespace pure_uav::app {
namespace {

constexpr int kLandTimeoutSeconds = 30;
constexpr double kLandAltitudeM = 0.10;
constexpr double kLandClimbMps = 0.10;

bool readAltitude(Communicator& comm, const MainConfig& cfg, const FlightState& state, double& altitude_m, double& climb_mps, std::string& error) {
    LocalPosition position;
    if (!comm.getLocalPosition(position, cfg.link.local_position_max_age_ms)) {
        error = "位置数据不新鲜: localPositionAgeMs=" + std::to_string(comm.localPositionAgeMs());
        return false;
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || !std::isfinite(position.vx) || !std::isfinite(position.vy) || !std::isfinite(position.vz)) {
        error = "位置或速度出现 NaN/Inf";
        return false;
    }
    altitude_m = -(static_cast<double>(position.z) - state.ground_z);
    climb_mps = -static_cast<double>(position.vz);
    return true;
}

bool switchToAutoLand(Communicator& comm, const MainConfig& cfg, Logger& log) {
    std::uint8_t ack = 0;
    log.info(Phase::kLand, "请求切换模式: AUTO_LAND");
    if (!comm.sendSetMode(px4_mode::AUTO_LAND, cfg.command.ack_timeout_ms, &ack)) {
        log.error(Phase::kLand, "切换 AUTO_LAND 未收到 ACK: mode=" + modeText(comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        log.error(Phase::kLand, "切换 AUTO_LAND 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(comm.getLastAck()) + " mode=" + modeText(comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(cfg.command.mode_timeout_s), [&comm] { return comm.getCustomMode() == px4_mode::AUTO_LAND; })) {
        log.error(Phase::kLand, "模式未生效: 期望=AUTO_LAND 当前=" + modeText(comm));
        return false;
    }
    log.info(Phase::kLand, "模式已切换: AUTO_LAND");
    return true;
}

bool waitLanded(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state, int timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    std::optional<std::chrono::steady_clock::time_point> stable_since;

    while (std::chrono::steady_clock::now() < deadline) {
        if (interrupted()) {
            log.warn(Phase::kLand, "等待落地时收到中断");
            return false;
        }
        if (!comm.isArmed()) {
            log.info(Phase::kLand, "飞控已自动解除武装");
            return true;
        }

        double altitude_m = 0.0;
        double climb_mps = 0.0;
        std::string height_error;
        if (readAltitude(comm, cfg, state, altitude_m, climb_mps, height_error)) {
            if (altitude_m <= kLandAltitudeM && std::fabs(climb_mps) <= kLandClimbMps) {
                if (!stable_since) {
                    stable_since = std::chrono::steady_clock::now();
                }
                if (std::chrono::steady_clock::now() - *stable_since >= std::chrono::seconds(1)) {
                    log.info(Phase::kLand, "已落地: alt=" + fixed(altitude_m) + "m");
                    return true;
                }
            } else {
                stable_since.reset();
            }
        } else {
            stable_since.reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    log.error(Phase::kLand, "降落超时: timeout=" + std::to_string(timeout_s) + "s armed=" + std::to_string(comm.isArmed()));
    return false;
}

bool disarm(Communicator& comm, const MainConfig& cfg, Logger& log) {
    if (!comm.isArmed()) {
        log.info(Phase::kDisarm, "已处于解除武装状态");
        return true;
    }

    std::uint8_t ack = 0;
    if (!comm.sendArm(false, cfg.command.ack_timeout_ms, &ack)) {
        log.error(Phase::kDisarm, "解除武装未收到 ACK: mode=" + modeText(comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        log.error(Phase::kDisarm, "解除武装被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(comm.getLastAck()));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(cfg.command.arm_timeout_s), [&comm] { return !comm.isArmed(); })) {
        log.error(Phase::kDisarm, "解除武装超时: armed=" + std::to_string(comm.isArmed()));
        return false;
    }

    log.info(Phase::kDisarm, "已解除武装");
    return true;
}

}  // namespace

OffboardZeroStreamer::OffboardZeroStreamer(Communicator& comm, int rate_hz) : comm_(comm), period_us_(1000000 / rate_hz) {
}

OffboardZeroStreamer::~OffboardZeroStreamer() {
    stop();
}

bool OffboardZeroStreamer::start(std::string& error) {
    if (running_.load()) {
        return true;
    }
    if (period_us_.count() <= 0) {
        error = "OFFBOARD 频率非法";
        return false;
    }
    running_.store(true);
    try {
        thread_ = std::thread(&OffboardZeroStreamer::run, this);
    } catch (const std::exception& e) {
        running_.store(false);
        error = std::string("OFFBOARD 推流线程创建失败: ") + e.what();
        return false;
    }
    return true;
}

void OffboardZeroStreamer::stop() {
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
}

std::uint64_t OffboardZeroStreamer::sent() const {
    return sent_.load();
}

std::uint64_t OffboardZeroStreamer::failed() const {
    return failed_.load();
}

bool OffboardZeroStreamer::running() const {
    return running_.load();
}

void OffboardZeroStreamer::run() {
    auto next = std::chrono::steady_clock::now();
    while (running_.load()) {
        GuidanceCmd cmd;
        cmd.mode = GuidanceMode::VELOCITY;
        cmd.vx = 0.0F;
        cmd.vy = 0.0F;
        cmd.vz = 0.0F;
        if (comm_.pushGuidanceCmd(cmd)) {
            sent_.fetch_add(1);
        } else {
            failed_.fetch_add(1);
        }
        next += period_us_;
        std::this_thread::sleep_until(next);
        if (next < std::chrono::steady_clock::now()) {
            next = std::chrono::steady_clock::now() + period_us_;
        }
    }
}

bool landAndDisarm(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state, OffboardZeroStreamer& streamer) {
    if (!comm.isArmed()) {
        log.info(Phase::kLand, "已解除武装，无需降落");
        streamer.stop();
        state.airborne = false;
        return true;
    }

    std::uint8_t ack = 0;
    bool land_sent = comm.sendLand(cfg.command.ack_timeout_ms, &ack);
    if (!land_sent || ack != MAV_RESULT_ACCEPTED) {
        log.warn(Phase::kLand, "NAV_LAND 未被接受，尝试 AUTO_LAND 模式");
        if (!switchToAutoLand(comm, cfg, log)) {
            log.error(Phase::kLand, "降落命令与 AUTO_LAND 模式均失败: mode=" + modeText(comm));
            return false;
        }
    } else if (!waitUntil(std::chrono::seconds(cfg.command.mode_timeout_s), [&comm] { return comm.getCustomMode() == px4_mode::AUTO_LAND; })) {
        log.warn(Phase::kLand, "NAV_LAND 后模式未变为 AUTO_LAND，尝试再次设置");
        if (!switchToAutoLand(comm, cfg, log)) {
            log.error(Phase::kLand, "无法进入 AUTO_LAND: mode=" + modeText(comm));
            return false;
        }
    }

    streamer.stop();
    log.info(Phase::kLand, "已进入自动降落，OFFBOARD 推流停止");

    if (!waitLanded(comm, cfg, log, state, kLandTimeoutSeconds)) {
        return false;
    }
    if (!disarm(comm, cfg, log)) {
        return false;
    }

    state.airborne = false;
    log.info(Phase::kLand, "降落并解除武装完成");
    return true;
}

void safeAbort(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state, OffboardZeroStreamer* streamer) {
    log.warn(Phase::kAbort, "进入安全退出");

    if (!comm.isConnected()) {
        log.error(Phase::kAbort, "链路已断，停止 OFFBOARD 推流，交给飞控 failsafe");
        if (streamer != nullptr) {
            streamer->stop();
        }
        return;
    }

    bool landed = !state.airborne;
    if (comm.isArmed() && state.airborne) {
        std::uint8_t ack = 0;
        if (!comm.sendLand(cfg.command.ack_timeout_ms, &ack) || ack != MAV_RESULT_ACCEPTED) {
            (void)comm.sendSetMode(px4_mode::AUTO_LAND, cfg.command.ack_timeout_ms, nullptr);
        }
        (void)waitUntil(std::chrono::seconds(cfg.command.mode_timeout_s), [&comm] { return comm.getCustomMode() == px4_mode::AUTO_LAND; });
        if (streamer != nullptr) {
            streamer->stop();
        }
        landed = waitLanded(comm, cfg, log, state, 10);
    }

    if (streamer != nullptr) {
        streamer->stop();
    }
    if (landed) {
        state.airborne = false;
    }
    if (comm.isArmed() && landed) {
        (void)disarm(comm, cfg, log);
    } else if (comm.isArmed()) {
        log.error(Phase::kAbort, "未确认落地，保持解锁状态，交给飞控/操作员处理");
    }
}

}  // namespace pure_uav::app
