#include "startup/flight_flow.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "common/mavlink.h"
#include "communicator.hpp"
#include "startup/flight_safety.hpp"
#include "startup/logging.hpp"
#include "startup/main_config.hpp"

namespace pure_uav::app {
namespace {

constexpr int kGroundReferenceSamples = 20;
constexpr int kGroundReferenceIntervalMs = 50;
constexpr double kMaxGroundReferenceStddevM = 0.05;
constexpr double kQuatNormTolerance = 1e-3;
constexpr double kRadToDeg = 57.29577951308232;
constexpr int kBatteryAlarmPercent = 20;

bool linkAlive(Communicator& comm, const MainConfig& cfg, std::string& error) {
    if (!comm.isConnected()) {
        error = "飞控链路断开";
        return false;
    }
    const std::int64_t heartbeat_age = comm.lastHeartbeatAgeMs();
    if (heartbeat_age < 0 || heartbeat_age > cfg.link.heartbeat_max_age_ms) {
        error = "心跳超时: lastHeartbeatAgeMs=" + std::to_string(heartbeat_age);
        return false;
    }
    if (comm.attitudeAgeMs() < 0 || comm.attitudeAgeMs() > cfg.link.attitude_max_age_ms) {
        error = "姿态数据不新鲜: attitudeAgeMs=" + std::to_string(comm.attitudeAgeMs());
        return false;
    }
    if (comm.localPositionAgeMs() < 0 || comm.localPositionAgeMs() > cfg.link.local_position_max_age_ms) {
        error = "位置数据不新鲜: localPositionAgeMs=" + std::to_string(comm.localPositionAgeMs());
        return false;
    }
    return true;
}

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

bool switchMode(Communicator& comm, const MainConfig& cfg, Logger& log, std::uint32_t custom_mode, const char* name, Phase phase) {
    std::uint8_t ack = 0;
    log.info(phase, std::string("请求切换模式: ") + name);
    if (!comm.sendSetMode(custom_mode, cfg.command.ack_timeout_ms, &ack)) {
        log.error(phase, std::string("切换 ") + name + " 未收到 ACK: mode=" + modeText(comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        log.error(phase, std::string("切换 ") + name + " 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(comm.getLastAck()) + " mode=" + modeText(comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(cfg.command.mode_timeout_s), [&comm, custom_mode] { return comm.getCustomMode() == custom_mode; })) {
        log.error(phase, std::string("模式未生效: 期望=") + name + " 当前=" + modeText(comm));
        return false;
    }

    log.info(phase, std::string("模式已切换: ") + name);
    return true;
}

}  // namespace

bool waitLink(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state) {
    const std::string path = findConfigFile(cfg.driver_config);
    CommConfig comm_cfg;
    if (!loadCommConfig(path, comm_cfg)) {
        log.error(Phase::kWaitLink, "通信配置加载失败: " + path);
        return false;
    }

    comm.setSystemId(2);
    comm.setComponentId(1);
    if (!comm.start(comm_cfg)) {
        log.error(Phase::kWaitLink, "串口打开失败: device=" + comm_cfg.device + " baud=" + std::to_string(comm_cfg.baudrate));
        return false;
    }

    log.info(Phase::kWaitLink, "串口已打开: device=" + comm_cfg.device + " baud=" + std::to_string(comm_cfg.baudrate));
    const auto start = std::chrono::steady_clock::now();
    auto next_log = start;
    bool requested_streams = false;

    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(cfg.link.timeout_s)) {
        if (interrupted()) {
            log.warn(Phase::kWaitLink, "等待链路时收到中断");
            return false;
        }

        const bool connected = comm.isConnected();
        const bool locked = comm.targetLocked();
        const bool px4 = comm.getAutopilot() == MAV_AUTOPILOT_PX4;
        const std::int64_t heartbeat_age = comm.lastHeartbeatAgeMs();
        const std::int64_t attitude_age = comm.attitudeAgeMs();
        const std::int64_t local_age = comm.localPositionAgeMs();
        const std::int64_t system_age = comm.systemTimeAgeMs();
        const bool battery_seen = comm.getCountBattery() > 0;

        const bool ready = connected && locked && px4 && heartbeat_age >= 0 && heartbeat_age <= cfg.link.heartbeat_max_age_ms && attitude_age >= 0 && attitude_age <= cfg.link.attitude_max_age_ms && local_age >= 0 && local_age <= cfg.link.local_position_max_age_ms && system_age >= 0 &&
                           system_age <= cfg.link.system_time_max_age_ms && battery_seen;
        if (ready) {
            state.reconnect_count = comm.reconnectCount();
            log.info(Phase::kWaitLink, "链路就绪: autopilot=" + std::string(comm.getAutopilotNameCn()) + " mode=" + modeText(comm));
            return true;
        }

        if (locked && !px4 && comm.getAutopilot() != 0) {
            log.error(Phase::kWaitLink, "固件不是 PX4: autopilot=" + std::string(comm.getAutopilotNameCn()));
            return false;
        }

        if (!requested_streams && std::chrono::steady_clock::now() - start > std::chrono::seconds(2) && (attitude_age < 0 || local_age < 0 || system_age < 0)) {
            comm.requestDefaultStreams();
            requested_streams = true;
            log.warn(Phase::kWaitLink, "遥测流未齐，已重新请求默认消息流");
        }

        if (std::chrono::steady_clock::now() >= next_log) {
            log.info(Phase::kWaitLink, "等待中: connected=" + std::to_string(connected) + " locked=" + std::to_string(locked) + " autopilot=" + std::string(comm.getAutopilotNameCn()) + " heartbeatAgeMs=" + std::to_string(heartbeat_age) + " attitudeAgeMs=" + std::to_string(attitude_age) +
                                           " localAgeMs=" + std::to_string(local_age) + " systemAgeMs=" + std::to_string(system_age) + " batterySeen=" + std::to_string(battery_seen));
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    log.error(Phase::kWaitLink, "链路超时: connected=" + std::to_string(comm.isConnected()) + " locked=" + std::to_string(comm.targetLocked()) + " autopilot=" + std::string(comm.getAutopilotNameCn()) + " heartbeatAgeMs=" + std::to_string(comm.lastHeartbeatAgeMs()) +
                                    " attitudeAgeMs=" + std::to_string(comm.attitudeAgeMs()) + " localAgeMs=" + std::to_string(comm.localPositionAgeMs()) + " systemAgeMs=" + std::to_string(comm.systemTimeAgeMs()) + " batteryCount=" + std::to_string(comm.getCountBattery()));
    return false;
}

constexpr std::size_t kPreflightCheckCount = 17;

struct CheckItem {
    const char* name = "";
    int ok = 0;
    std::string reason;
};

using PreflightChecks = std::array<CheckItem, kPreflightCheckCount>;

void setCheck(PreflightChecks& checks, std::size_t index, const char* name, bool ok, std::string reason) {
    checks[index] = CheckItem{name, ok ? 1 : 0, ok ? std::string{} : std::move(reason)};
}

bool runPreflight(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state) {
    Attitude attitude;
    LocalPosition position;
    Battery battery;
    SystemTime system_time;

    const bool attitude_ok = comm.getAttitude(attitude, cfg.link.attitude_max_age_ms);
    const bool position_ok = comm.getLocalPosition(position, cfg.link.local_position_max_age_ms);
    const bool system_time_ok = comm.getSystemTime(system_time, cfg.link.system_time_max_age_ms);
    const bool battery_ok = comm.getCountBattery() > 0;
    comm.getBattery(battery);

    const bool quat_finite = std::isfinite(attitude.q[0]) && std::isfinite(attitude.q[1]) && std::isfinite(attitude.q[2]) && std::isfinite(attitude.q[3]);
    const double quat_norm = quat_finite ? std::sqrt(attitude.q[0] * attitude.q[0] + attitude.q[1] * attitude.q[1] + attitude.q[2] * attitude.q[2] + attitude.q[3] * attitude.q[3]) : 0.0;
    const bool attitude_finite = attitude_ok && std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) && std::isfinite(attitude.yaw) && quat_finite && std::fabs(quat_norm - 1.0) <= kQuatNormTolerance;
    const bool attitude_level = attitude_ok && std::fabs(static_cast<double>(attitude.roll)) * kRadToDeg <= cfg.preflight.max_tilt_deg && std::fabs(static_cast<double>(attitude.pitch)) * kRadToDeg <= cfg.preflight.max_tilt_deg;

    const bool position_finite = position_ok && std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) && std::isfinite(position.vx) && std::isfinite(position.vy) && std::isfinite(position.vz);
    const bool vertical_speed_ok = position_ok && std::fabs(static_cast<double>(position.vz)) <= cfg.preflight.max_vertical_speed_mps;

    const std::uint32_t mode = comm.getCustomMode();
    const bool mode_safe = mode != 0 && mode != px4_mode::OFFBOARD && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LAND && mode != px4_mode::AUTO_MISSION && mode != px4_mode::AUTO_RTL;

    PreflightChecks checks{};
    setCheck(checks, 0, "link_connected", comm.isConnected(), "isConnected=0");
    setCheck(checks, 1, "target_locked", comm.targetLocked(), "targetLocked=0");
    setCheck(checks, 2, "autopilot_px4", comm.getAutopilot() == MAV_AUTOPILOT_PX4, "autopilot=" + std::string(comm.getAutopilotNameCn()));
    setCheck(checks, 3, "heartbeat_fresh", comm.lastHeartbeatAgeMs() >= 0 && comm.lastHeartbeatAgeMs() <= cfg.link.heartbeat_max_age_ms, "lastHeartbeatAgeMs=" + std::to_string(comm.lastHeartbeatAgeMs()));
    setCheck(checks, 4, "attitude_fresh", attitude_ok, "attitudeAgeMs=" + std::to_string(comm.attitudeAgeMs()));
    setCheck(checks, 5, "local_position_fresh", position_ok, "localPositionAgeMs=" + std::to_string(comm.localPositionAgeMs()));
    setCheck(checks, 6, "system_time_fresh", system_time_ok, "systemTimeAgeMs=" + std::to_string(comm.systemTimeAgeMs()));
    setCheck(checks, 7, "not_armed", !comm.isArmed(), "isArmed=1");
    setCheck(checks, 8, "mode_safe", mode_safe, "mode=" + modeText(comm));
    setCheck(checks, 9, "battery_received", battery_ok, "batteryCount=" + std::to_string(comm.getCountBattery()));
    setCheck(checks, 10, "battery_voltage_valid", battery_ok && std::isfinite(battery.voltage) && battery.voltage > 0.0F, "voltage=" + fixed(battery.voltage));
    setCheck(checks, 11, "battery_level_ok", battery_ok && battery.remaining >= cfg.preflight.min_battery_percent, "remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "% < " + std::to_string(cfg.preflight.min_battery_percent) + "%");
    setCheck(checks, 12, "attitude_finite", attitude_finite, "attitude 含 NaN/Inf 或四元数模长异常");
    setCheck(checks, 13, "attitude_level", attitude_level, "roll=" + fixed(static_cast<double>(attitude.roll) * kRadToDeg) + "deg pitch=" + fixed(static_cast<double>(attitude.pitch) * kRadToDeg) + "deg");
    setCheck(checks, 14, "local_position_finite", position_finite, "位置或速度含 NaN/Inf");
    setCheck(checks, 15, "vertical_speed_small", vertical_speed_ok, "vz=" + fixed(static_cast<double>(position.vz)));
    setCheck(checks, 16, "reconnect_stable", state.reconnect_count == comm.reconnectCount(), "reconnectCount=" + std::to_string(comm.reconnectCount()));

    const bool pass = std::all_of(checks.begin(), checks.end(), [](const CheckItem& item) { return item.ok == 1; });
    for (const auto& item : checks) {
        if (item.ok == 0) {
            log.error(Phase::kPreflight, std::string(item.name) + ": " + item.reason);
        }
    }

    if (pass) {
        log.info(Phase::kPreflight, "自检通过 17/17");
    } else {
        log.error(Phase::kPreflight, "自检失败，禁止解锁");
    }
    return pass;
}

bool captureGroundReference(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state) {
    double sum = 0.0;
    double sum_sq = 0.0;
    int count = 0;

    for (int i = 0; i < kGroundReferenceSamples; ++i) {
        if (interrupted()) {
            log.warn(Phase::kGroundReference, "采集地面参考时收到中断");
            return false;
        }
        LocalPosition position;
        if (!comm.getLocalPosition(position, cfg.link.local_position_max_age_ms) || !std::isfinite(position.z)) {
            log.error(Phase::kGroundReference, "地面参考读取失败: localPositionAgeMs=" + std::to_string(comm.localPositionAgeMs()));
            return false;
        }
        sum += static_cast<double>(position.z);
        sum_sq += static_cast<double>(position.z) * static_cast<double>(position.z);
        ++count;
        if (!sleepInterruptible(kGroundReferenceIntervalMs)) {
            log.warn(Phase::kGroundReference, "采集地面参考时收到中断");
            return false;
        }
    }

    if (count != kGroundReferenceSamples) {
        log.error(Phase::kGroundReference, "地面参考样本不足: " + std::to_string(count));
        return false;
    }
    const double mean = sum / static_cast<double>(count);
    const double variance = std::max(0.0, sum_sq / static_cast<double>(count) - mean * mean);
    const double stddev = std::sqrt(variance);
    if (stddev > kMaxGroundReferenceStddevM) {
        log.error(Phase::kGroundReference, "地面参考抖动过大: stddev=" + fixed(stddev, 3) + "m");
        return false;
    }

    state.ground_z = mean;
    log.info(Phase::kGroundReference, "地面参考已锁定: z=" + fixed(state.ground_z, 3) + "m stddev=" + fixed(stddev, 3) + "m");
    return true;
}

bool arm(Communicator& comm, const MainConfig& cfg, Logger& log) {
    std::uint8_t ack = 0;
    if (!comm.sendArm(true, cfg.command.ack_timeout_ms, &ack)) {
        log.error(Phase::kArm, "解锁未收到 ACK: mode=" + modeText(comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        log.error(Phase::kArm, "解锁被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(comm.getLastAck()) + " mode=" + modeText(comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(cfg.command.arm_timeout_s), [&comm] { return comm.isArmed(); })) {
        log.error(Phase::kArm, "ACK 已接受但未进入解锁状态: armed=" + std::to_string(comm.isArmed()) + " mode=" + modeText(comm));
        return false;
    }

    log.info(Phase::kArm, "已解锁: mode=" + modeText(comm));
    return true;
}

bool takeoff(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state) {
    if (!switchMode(comm, cfg, log, px4_mode::AUTO_TAKEOFF, "AUTO_TAKEOFF", Phase::kTakeoffMode)) {
        return false;
    }

    std::uint8_t ack = 0;
    if (!comm.sendTakeoff(static_cast<float>(cfg.takeoff.altitude_m), cfg.command.ack_timeout_ms, &ack)) {
        log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 未收到 ACK: mode=" + modeText(comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(comm.getLastAck()) + " mode=" + modeText(comm));
        return false;
    }

    state.airborne = true;
    log.info(Phase::kTakeoffCommand, "起飞命令已接受: target=" + fixed(cfg.takeoff.altitude_m) + "m");
    return true;
}

bool waitAltitude(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state) {
    const double target = cfg.takeoff.altitude_m;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(cfg.takeoff.timeout_s);
    auto next_log = std::chrono::steady_clock::now();
    std::optional<std::chrono::steady_clock::time_point> stable_since;

    while (std::chrono::steady_clock::now() < deadline) {
        if (interrupted()) {
            log.warn(Phase::kClimb, "爬升时收到中断");
            return false;
        }
        std::string link_error;
        if (!linkAlive(comm, cfg, link_error)) {
            log.error(Phase::kClimb, link_error);
            return false;
        }
        if (!comm.isArmed()) {
            log.error(Phase::kClimb, "爬升过程中检测到已解除武装");
            return false;
        }

        double altitude_m = 0.0;
        double climb_mps = 0.0;
        std::string height_error;
        const bool height_ok = readAltitude(comm, cfg, state, altitude_m, climb_mps, height_error);
        const std::uint32_t mode = comm.getCustomMode();

        if (height_ok && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LOITER) {
            log.error(Phase::kClimb, "爬升模式异常: mode=" + modeText(comm));
            return false;
        }

        const bool near_target = height_ok && std::fabs(altitude_m - target) <= cfg.takeoff.height_tolerance_m && std::fabs(climb_mps) <= cfg.preflight.max_vertical_speed_mps;
        if (near_target) {
            if (!stable_since) {
                stable_since = std::chrono::steady_clock::now();
            }
            if (std::chrono::steady_clock::now() - *stable_since >= std::chrono::duration<double>(cfg.takeoff.height_stable_s)) {
                log.info(Phase::kClimb, "到达目标高度: alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(comm));
                return true;
            }
        } else {
            stable_since.reset();
        }

        if (std::chrono::steady_clock::now() >= next_log) {
            if (height_ok) {
                log.info(Phase::kClimb, "alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(comm));
            } else {
                log.warn(Phase::kClimb, height_error);
            }
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    log.error(Phase::kClimb, "爬升超时: target=" + fixed(target) + "m timeout=" + std::to_string(cfg.takeoff.timeout_s) + "s");
    return false;
}

bool hover(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state) {
    if (!switchMode(comm, cfg, log, px4_mode::AUTO_LOITER, "AUTO_LOITER", Phase::kHover)) {
        return false;
    }
    if (!sleepInterruptible(1000)) {
        log.warn(Phase::kHover, "悬停稳定等待时收到中断");
        return false;
    }

    std::string link_error;
    if (!linkAlive(comm, cfg, link_error)) {
        log.error(Phase::kHover, link_error);
        return false;
    }
    if (!comm.isArmed()) {
        log.error(Phase::kHover, "悬停过程中检测到已解除武装");
        return false;
    }

    double altitude_m = 0.0;
    double climb_mps = 0.0;
    std::string height_error;
    if (!readAltitude(comm, cfg, state, altitude_m, climb_mps, height_error)) {
        log.error(Phase::kHover, height_error);
        return false;
    }
    if (std::fabs(altitude_m - cfg.takeoff.altitude_m) > cfg.takeoff.height_tolerance_m) {
        log.error(Phase::kHover, "悬停高度偏差过大: alt=" + fixed(altitude_m) + "m target=" + fixed(cfg.takeoff.altitude_m) + "m");
        return false;
    }

    log.info(Phase::kHover, "AUTO_LOITER 稳定: alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(comm));
    return true;
}

bool enterOffboard(Communicator& comm, const MainConfig& cfg, Logger& log, OffboardZeroStreamer& streamer) {
    std::string start_error;
    if (!streamer.start(start_error)) {
        log.error(Phase::kOffboard, start_error);
        return false;
    }

    const int prestream_ms = static_cast<int>(cfg.offboard.prestream_s * 1000.0);
    if (!sleepInterruptible(prestream_ms)) {
        log.warn(Phase::kOffboard, "OFFBOARD 预推流时收到中断");
        streamer.stop();
        return false;
    }
    if (streamer.failed() > 0 || streamer.sent() == 0) {
        log.error(Phase::kOffboard, "OFFBOARD 预推流失败: sent=" + std::to_string(streamer.sent()) + " failed=" + std::to_string(streamer.failed()));
        streamer.stop();
        return false;
    }

    if (!switchMode(comm, cfg, log, px4_mode::OFFBOARD, "OFFBOARD", Phase::kOffboard)) {
        log.warn(Phase::kOffboard, "切 OFFBOARD 失败，尝试退回 AUTO_LOITER");
        (void)switchMode(comm, cfg, log, px4_mode::AUTO_LOITER, "AUTO_LOITER", Phase::kHover);
        streamer.stop();
        return false;
    }

    log.info(Phase::kOffboard, "已进入 OFFBOARD：零速度保持，后续 guidance 线程/状态机未启动");
    return true;
}

bool holdOffboard(Communicator& comm, const MainConfig& cfg, Logger& log, OffboardZeroStreamer& streamer, const FlightState& state) {
    auto next_log = std::chrono::steady_clock::now();
    while (!interrupted()) {
        std::string link_error;
        if (!linkAlive(comm, cfg, link_error)) {
            log.error(Phase::kOffboard, link_error);
            return false;
        }
        if (!comm.isArmed()) {
            log.error(Phase::kOffboard, "OFFBOARD 保持期间检测到已解除武装");
            return false;
        }
        if (comm.getCustomMode() != px4_mode::OFFBOARD) {
            log.error(Phase::kOffboard, "OFFBOARD 模式丢失: mode=" + modeText(comm));
            return false;
        }
        if (streamer.failed() > 0) {
            log.error(Phase::kOffboard, "OFFBOARD setpoint 发送失败: failed=" + std::to_string(streamer.failed()));
            return false;
        }

        Battery battery;
        comm.getBattery(battery);
        if (battery.remaining >= 0 && battery.remaining < kBatteryAlarmPercent) {
            log.error(Phase::kOffboard, "电量过低: remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "%");
            return false;
        }

        if (std::chrono::steady_clock::now() >= next_log) {
            double altitude_m = 0.0;
            double climb_mps = 0.0;
            std::string height_error;
            if (readAltitude(comm, cfg, state, altitude_m, climb_mps, height_error)) {
                log.info(Phase::kOffboard, "alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s sent=" + std::to_string(streamer.sent()));
            } else {
                log.warn(Phase::kOffboard, height_error);
            }
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }
    return true;
}

}  // namespace pure_uav::app
