#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "common/mavlink.h"
#include "detection_process.hpp"
#include "glue/mission_ports.hpp"
#include "mission_target_channel.hpp"

namespace {
using namespace pure_uav;
using namespace pure_uav::app;
using Clock = std::chrono::steady_clock;
void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
void writeMessage(int fd, const mavlink_message_t& message) {
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    check(write(fd, bytes.data(), length) == length, "PTY frame write failed");
}
std::vector<mavlink_message_t> readMessages(int fd) {
    std::vector<mavlink_message_t> messages;
    mavlink_message_t parser{};
    mavlink_message_t message{};
    mavlink_status_t status{};
    mavlink_status_t result{};
    const auto deadline = Clock::now() + std::chrono::milliseconds(80);
    while (Clock::now() < deadline) {
        pollfd item{fd, POLLIN, 0};
        if (poll(&item, 1, 5) <= 0) {
            continue;
        }
        uint8_t bytes[4096];
        const auto count = read(fd, bytes, sizeof(bytes));
        for (ssize_t i = 0; i < count; ++i) {
            if (mavlink_frame_char_buffer(&parser, &status, bytes[i], &message, &result) == MAVLINK_FRAMING_OK) {
                messages.push_back(message);
            }
        }
    }
    return messages;
}
void targetIpc() {
    MissionTargetChannel channel;
    check(channel.open(), "target channel open failed");
    MissionTargetPacket packet;
    const auto now = nowNs();
    packet.captured_ns = now - 1000000;
    packet.present = 1;
    packet.u_px = 100;
    packet.v_px = 200;
    packet.w_px = 20;
    packet.h_px = 30;
    packet.confidence = 0.9;
    packet.class_id = 1;
    check(publishMissionTarget(channel.publisherFd(), packet), "target publish failed");
    MissionTargetPacket received;
    check(channel.latest(received, now, 200000000) && received.u_px == 100 && received.has_range == 0, "target data or missing-range semantics lost");
    check(publishMissionTarget(channel.publisherFd(), packet), "duplicate publish failed");
    check(!channel.latest(received, now, 200000000), "duplicate capture accepted");
    packet.captured_ns = now + 1;
    publishMissionTarget(channel.publisherFd(), packet);
    check(!channel.latest(received, now, 200000000), "future target accepted");
    packet.captured_ns = now - 500000000;
    publishMissionTarget(channel.publisherFd(), packet);
    check(!channel.latest(received, now, 200000000), "stale target accepted");
    packet.captured_ns = now;
    packet.version = 9;
    publishMissionTarget(channel.publisherFd(), packet);
    check(!channel.latest(received, now, 200000000), "unknown target protocol accepted");

    DetectionProcessOptions options;
    options.executable = "/proc/self/exe";
    options.arguments = {"--publisher", std::to_string(channel.publisherFd())};
    options.target_publish_fd = channel.publisherFd();
    options.poll_interval_ms = 5;
    DetectionProcess process(options);
    for (int attempt = 0; attempt < 2; ++attempt) {
        std::string error;
        check(process.start(&error), "target publisher exec failed");
        check(process.supervise([] { return false; }).status == SuperviseStatus::exited, "target publisher failed after exec");
        check(channel.latest(received, nowNs(), 200000000) && received.sequence == 42, "exec/restart lost inherited target fd");
    }
}
void mavlinkWire() {
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    check(master >= 0, "PTY master failed");
    struct Cleanup {
        int fd;
        ~Cleanup() {
            close(fd);
        }
    } cleanup{master};
    check(grantpt(master) == 0 && unlockpt(master) == 0, "PTY setup failed");
    Communicator comm;
    CommConfig config;
    config.device = ptsname(master);
    config.heartbeat_timeout_ms = 3000;
    check(comm.start(config), "Communicator start failed");
    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, px4_mode::POSCTL, MAV_STATE_STANDBY);
    writeMessage(master, message);
    auto deadline = Clock::now() + std::chrono::seconds(1);
    while (!comm.isConnected() && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(comm.isConnected(), "heartbeat not received");
    mavlink_msg_extended_sys_state_pack(1, 1, &message, MAV_VTOL_STATE_UNDEFINED, MAV_LANDED_STATE_ON_GROUND);
    writeMessage(master, message);
    mavlink_msg_attitude_pack(1, 1, &message, 1000, 0, 0, 0, 0, 0, 0);
    writeMessage(master, message);
    mavlink_msg_local_position_ned_pack(1, 1, &message, 1000, 2, 3, -0.4F, 0, 0, 0);
    writeMessage(master, message);
    MavlinkMissionPort port(comm, 0.2, 3000);
    mission::FlightFeedback feedback;
    deadline = Clock::now() + std::chrono::seconds(1);
    do {
        feedback = port.feedback(static_cast<double>(nowNs()) * 1e-9);
        if (feedback.input.landed && feedback.input.ins.has_position) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (Clock::now() < deadline);
    check(feedback.supported && feedback.input.landed && feedback.input.ins.has_position && !feedback.input.armed, "mission feedback did not bridge real MAVLink");
    const auto pose_time = feedback.input.pose_time_s;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    check(port.feedback(static_cast<double>(nowNs()) * 1e-9).input.pose_time_s == pose_time, "reading cached pose refreshed its timestamp");
    const auto before = Clock::now();
    check(port.request(mission::FlightRequest::kOffboard, feedback.session), "nonblocking offboard failed");
    check(port.request(mission::FlightRequest::kArm, feedback.session), "nonblocking arm failed");
    check(port.request(mission::FlightRequest::kLand, feedback.session), "nonblocking land failed");
    check(Clock::now() - before < std::chrono::milliseconds(200), "flight requests waited for absent ACKs");
    GuidanceCmd command;
    command.mode = GuidanceMode::POSITION;
    command.x = 2;
    command.y = 3;
    command.z = -1.9F;
    check(port.send(command, feedback.session), "mission setpoint failed");
    check(!port.send(command, feedback.session + 1) && !port.request(mission::FlightRequest::kArm, feedback.session + 1), "stale session sent flight control");
    bool mode = false;
    bool arm = false;
    bool land = false;
    bool position = false;
    for (const auto& packet : readMessages(master)) {
        if (packet.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            mavlink_command_long_t value;
            mavlink_msg_command_long_decode(&packet, &value);
            if (value.command == MAV_CMD_DO_SET_MODE) {
                mode = value.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED && value.param2 == 6;
            }
            if (value.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                arm = value.param1 == 1 && value.param2 == 0;
            }
            if (value.command == MAV_CMD_NAV_LAND) {
                land = true;
            }
        }
        if (packet.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
            mavlink_set_position_target_local_ned_t value;
            mavlink_msg_set_position_target_local_ned_decode(&packet, &value);
            position = value.x == 2 && value.y == 3 && std::abs(value.z + 1.9F) < 1e-6F && value.coordinate_frame == MAV_FRAME_LOCAL_NED;
        }
    }
    check(mode && arm && land && position, "mission output did not reach MAVLink wire correctly");
    comm.stop();
    LandedState state;
    check(!comm.getLandedState(state), "stopped session retained landed telemetry");
}
}  // namespace
int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--publisher") {
        MissionTargetPacket packet;
        packet.captured_ns = nowNs();
        packet.sequence = 42;
        return publishMissionTarget(std::stoi(argv[2]), packet) ? 0 : 1;
    }
    try {
        targetIpc();
        mavlinkWire();
        std::puts("PASS: detection IPC/exec/restart and mission MAVLink feedback/nonblocking output/session guards");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
