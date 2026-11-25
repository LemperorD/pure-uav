#include "pty_test_helpers.hpp"
#ifdef TEST_COMMUNICATOR
#include "communicator.hpp"
using Transport = pure_uav::Communicator;
#else
#include "mavlink_link.hpp"
using Transport = pure_uav::MavlinkLink;
#endif
using namespace wire_test;

int main() {
    try {
        Pty pty;
        DevicePath device(pty.slave);
        Transport link;
#ifdef TEST_COMMUNICATOR
        pure_uav::CommConfig cfg;
        cfg.device = device.path;
        cfg.fcu_sysid = 9;
        cfg.fcu_compid = 42;
        cfg.heartbeat_timeout_ms = 250;
        check(link.start(cfg), "start communicator");
        auto connected = [&] { return link.isConnected(); };
        auto armed = [&] { return link.isArmed(); };
        auto publish = [&] {
            pure_uav::VisionPose pose{};
            return link.pushVisionPose(pose);
        };
#else
        Transport::Config cfg;
        cfg.device = device.path;
        cfg.target_sysid = 9;
        cfg.target_compid = 42;
        cfg.heartbeat_timeout_ms = 250;
        cfg.send_heartbeat = false;
        std::string error;
        check(link.start(cfg, &error), "start link");
        auto connected = [&] { return link.connected(); };
        auto armed = [&] { return link.armed(); };
        auto publish = [&] {
            pure_uav::VisionPose pose{};
            return link.sendVisionPose(pose);
        };
#endif
        check(!publish(), "disconnected publication must fail");
        imu(pty.master, 1000000, 8, 42);
        imu(pty.master, 1005000);  // Even configured target data needs a confirming heartbeat.
        std::this_thread::sleep_for(milliseconds(40));
        check(link.receivedImu() == 0 && link.imuQueueSize() == 0, "pre-heartbeat telemetry accepted");
        heartbeat(pty.master, 9, 42, true, MAV_AUTOPILOT_INVALID);
        std::this_thread::sleep_for(milliseconds(20));
        check(!connected(), "non-autopilot heartbeat established connection");
        heartbeat(pty.master);
        check(waitFor(connected), "target heartbeat did not connect");
        check(armed(), "armed state missing");
        imu(pty.master, 1010000, 9, 43);
        imu(pty.master, 1015000, 8, 42);
        imu(pty.master, 1020000);
        check(waitFor([&] { return link.receivedImu() == 1; }), "valid telemetry missing");
        Transport::TimedImu first{};
        check(link.popImu(first) && first.imu.timestamp_us == 1020000, "foreign component polluted queue");
        imu(pty.master, 1025000);
        check(waitFor([&] { return link.imuQueueSize() == 1; }), "pending IMU missing");
        const auto generation = link.reconnectCount();
        check(waitFor([&] { return !connected() && link.reconnectCount() > generation; }), "timeout did not invalidate session");
        check(link.imuQueueSize() == 0 && armed(), "timeout kept stale IMU or erased armed state");
#ifndef TEST_COMMUNICATOR
        check(link.lastImuTimestampUs() == 0, "timeout retained last IMU timestamp");
#else
        pure_uav::ImuData cached{};
        link.getImu(cached);
        check(cached.timestamp_us == 0, "timeout retained cached IMU timestamp");
#endif
        check(!publish(), "publication allowed after timeout");
        const auto count = link.receivedImu();
        imu(pty.master, 1030000, 8, 42);
        imu(pty.master, 1035000);
        std::this_thread::sleep_for(milliseconds(30));
        check(link.receivedImu() == count && link.imuQueueSize() == 0, "timeout opened telemetry admission");
        heartbeat(pty.master);
        check(waitFor(connected), "heartbeat reacquisition failed");
        imu(pty.master, 1000000);
        check(waitFor([&] { return link.imuQueueSize() == 1; }), "new-session telemetry missing");
        Transport::TimedImu second{};
        check(link.popImu(second) && second.session > first.session, "IMU has no session boundary");
        // 物理设备挂断与同路径重开：不依赖实际 USB，不向任何真飞控发命令。
        const auto before_reopen = link.reconnectCount();
        ::close(pty.master);
        pty.master = -1;
        check(waitFor([&] { return !connected(); }), "device hangup not detected");
        check(link.imuQueueSize() == 0 && armed(), "device loss kept IMU or cleared armed state");
        Pty replacement;
        device.pointTo(replacement.slave);
        check(waitFor([&] { return link.reconnectCount() > before_reopen; }, 2500), "device did not reopen");
        check(waitFor([&] {
                  heartbeat(replacement.master);
                  return connected();
              }),
              "reopened heartbeat missing");
        imu(replacement.master, 500000);
        check(waitFor([&] { return link.imuQueueSize() == 1; }), "reopened IMU missing");
        Transport::TimedImu third{};
        check(link.popImu(third) && third.session > second.session && third.imu.timestamp_us == 500000, "reopened queue retained old session");
        link.stop();
        std::puts("PASS: session admission, timeout isolation and reacquisition");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
