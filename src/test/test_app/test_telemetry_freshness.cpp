#include <cstdio>
#include <fstream>

#include "communicator.hpp"
#include "pty_test_helpers.hpp"

using namespace wire_test;
namespace {
void telemetry(int fd, uint32_t stamp) {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(9, 42, &message, stamp, 0.25f, -0.5f, 1.0f, 0, 0, 0);
    send(fd, message);
    mavlink_msg_local_position_ned_pack(9, 42, &message, stamp, 10, 20, -30, 1, 2, -3);
    send(fd, message);
    mavlink_msg_system_time_pack(9, 42, &message, 1700000000000000ULL + stamp, stamp);
    send(fd, message);
}
void absent(pure_uav::Communicator& comm) {
    pure_uav::Attitude attitude;
    pure_uav::LocalPosition position;
    pure_uav::SystemTime time;
    attitude.yaw = 42;
    position.x = 42;
    time.time_boot_ms = 42;
    check(!comm.getAttitude(attitude) && !comm.getLocalPosition(position) && !comm.getSystemTime(time), "old or absent telemetry accepted");
    check(attitude.yaw == 42 && position.x == 42 && time.time_boot_ms == 42, "failed getter overwrote caller output");
    check(comm.attitudeAgeMs() == -1 && comm.localPositionAgeMs() == -1 && comm.systemTimeAgeMs() == -1, "invalidated telemetry has an age");
}
void present(pure_uav::Communicator& comm, uint32_t stamp) {
    pure_uav::Attitude attitude;
    pure_uav::LocalPosition position;
    pure_uav::SystemTime time;
    check(waitFor([&] { return comm.getAttitude(attitude, 200) && comm.getLocalPosition(position, 200) && comm.getSystemTime(time, 200); }), "fresh telemetry missing");
    check(attitude.time_boot_ms == stamp && position.time_boot_ms == stamp && time.time_boot_ms == stamp, "telemetry belongs to a previous session");
    check(comm.attitudeAgeMs() >= 0 && comm.attitudeAgeMs() < 200 && comm.localPositionAgeMs() >= 0 && comm.localPositionAgeMs() < 200 && comm.systemTimeAgeMs() >= 0 && comm.systemTimeAgeMs() < 200, "invalid telemetry age");
}
}  // namespace
int main() {
    try {
        Pty pty;
        DevicePath device(pty.slave);
        const std::string config_file = device.directory + "/driver.yaml";
        {
            std::ofstream config(config_file);
            config << "communicate:\n  file: " << device.path << "\n  fcu_sysid: 9\n  fcu_compid: 42\n"
                   << "  heartbeat_timeout_ms: 250\n";
        }
        pure_uav::CommConfig cfg;
        check(pure_uav::loadCommConfig(config_file, cfg) && cfg.fcu_sysid == 9 && cfg.fcu_compid == 42, "loadCommConfig ignored FCU identity");
        {
            std::ofstream config(config_file);
            config << "communicate:\n  fcu_sysid: 256\n";
        }
        pure_uav::CommConfig invalid;
        check(!pure_uav::loadCommConfig(config_file, invalid), "invalid FCU identity accepted");
        ::unlink(config_file.c_str());
        pure_uav::Communicator comm;
        check(comm.start(cfg), "start communicator");
        absent(comm);
        heartbeat(pty.master);
        check(waitFor([&] { return comm.isConnected(); }), "heartbeat missing");
        absent(comm);
        telemetry(pty.master, 1000);
        present(comm, 1000);
        std::this_thread::sleep_for(milliseconds(35));
        pure_uav::Attitude attitude;
        pure_uav::LocalPosition position;
        pure_uav::SystemTime time;
        check(!comm.getAttitude(attitude, 5) && !comm.getLocalPosition(position, 5) && !comm.getSystemTime(time, 5), "expired telemetry accepted with max_age");
        check(comm.getAttitude(attitude) && comm.getLocalPosition(position) && comm.getSystemTime(time), "default getter must accept current-session older telemetry");
        const auto session = comm.reconnectCount();
        check(waitFor([&] { return !comm.isConnected() && comm.reconnectCount() > session; }), "heartbeat timeout missing");
        absent(comm);
        heartbeat(pty.master);
        check(waitFor([&] { return comm.isConnected(); }), "heartbeat reacquisition");
        absent(comm);
        telemetry(pty.master, 2000);
        present(comm, 2000);
        const auto old_session = comm.reconnectCount();
        ::close(pty.master);
        pty.master = -1;
        check(waitFor([&] { return !comm.isConnected(); }), "PTY hangup missing");
        absent(comm);
        Pty replacement;
        device.pointTo(replacement.slave);
        check(waitFor([&] { return comm.reconnectCount() > old_session; }, 2500), "reopen missing");
        absent(comm);
        check(waitFor([&] {
                  heartbeat(replacement.master);
                  return comm.isConnected();
              }),
              "reopened heartbeat");
        absent(comm);
        telemetry(replacement.master, 3000);
        present(comm, 3000);
        comm.stop();
        absent(comm);
        check(comm.start(cfg), "restart communicator");
        absent(comm);
        comm.stop();
        std::puts("PASS: telemetry session freshness, ages, optional expiration, timeout, hangup, reopen, identity config");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
