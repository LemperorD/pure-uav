#include <future>
#include "pty_test_helpers.hpp"
#include "communicator.hpp"
using namespace wire_test;
int main() {
    try {
        Pty pty; pure_uav::Communicator link; pure_uav::CommConfig cfg;
        cfg.device = pty.slave; cfg.fcu_sysid = 9; cfg.fcu_compid = 42;
        cfg.heartbeat_timeout_ms = 5000;
        check(link.start(cfg), "start");
        heartbeat(pty.master); check(waitFor([&] { return link.isConnected(); }), "connect");
        Reader reader(pty.master); reader.drain();
        auto command = [&](uint16_t expected, auto invoke, float arm_value = -1) {
            uint8_t result = UINT8_MAX;
            auto future = std::async(std::launch::async, [&] { return invoke(&result); });
            bool captured = false;
            mavlink_message_t message{};
            const auto deadline = steady_clock::now() + milliseconds(800);
            while (steady_clock::now() < deadline && reader.next(message, 50)) {
                if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) continue;
                mavlink_command_long_t cmd{}; mavlink_msg_command_long_decode(&message, &cmd);
                if (cmd.command != expected) continue;
                check(cmd.target_system == 9 && cmd.target_component == 42, "command used wrong FC identity");
                if (arm_value >= 0) {
                    check(cmd.param1 == arm_value, "wrong arm/disarm value");
                    check(cmd.param2 == 0.0f, "ordinary arm/disarm forced safety override");
                }
                mavlink_message_t ack{};
                mavlink_msg_command_ack_pack(9, 42, &ack, expected, MAV_RESULT_ACCEPTED, 0, 0, 2, 1);
                send(pty.master, ack); captured = true; break;
            }
            check(captured, "command not captured");
            check(future.get() && result == MAV_RESULT_ACCEPTED, "command ACK failed");
        };
        command(MAV_CMD_COMPONENT_ARM_DISARM, [&](uint8_t* r) { return link.sendArm(true, 1000, r); }, 1);
        command(MAV_CMD_COMPONENT_ARM_DISARM, [&](uint8_t* r) { return link.sendArm(false, 1000, r); }, 0);
        command(MAV_CMD_NAV_TAKEOFF, [&](uint8_t* r) { return link.sendTakeoff(1, 1000, r); });
        command(MAV_CMD_NAV_LAND, [&](uint8_t* r) { return link.sendLand(1000, r); });
        command(MAV_CMD_DO_SET_MODE, [&](uint8_t* r) { return link.sendSetMode(4, 1000, r); });
        pure_uav::GuidanceCmd cmd{}; check(link.pushGuidanceCmd(cmd), "guidance send");
        bool guidance = false; mavlink_message_t message{};
        while (reader.next(message, 100)) {
            if (message.msgid != MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) continue;
            mavlink_set_position_target_local_ned_t target{};
            mavlink_msg_set_position_target_local_ned_decode(&message, &target);
            check(target.target_system == 9 && target.target_component == 42, "guidance wrong target");
            guidance = true; break;
        }
        check(guidance, "guidance command not captured");
        link.stop(); std::puts("PASS: nondefault FC command targets and normal arm/disarm"); return 0;
    } catch (const std::exception& e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
