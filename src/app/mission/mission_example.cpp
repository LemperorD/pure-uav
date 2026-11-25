// 仅验证任务调度：反馈和撞击事件都是合成的，不连接串口/相机，不仿真制导动力学。
#include <cstdio>

#include "mission/mission_controller.hpp"

int main() {
    using namespace pure_uav;
    using namespace pure_uav::app::mission;
    MissionController mission;
    MissionInput input;
    input.fcu_connected = input.preflight_ready = input.landed = true;
    input.ins.has_position = input.ins.has_attitude = true;
    input.ins.p = {2.0, 3.0, -0.4};  // 故意使用非零原点，验证相对高度和返航点。
    input.guidance_ready = true;
    double state_entered_s = 0.0;
    double prestream_started_s = -1.0;
    for (int tick = 0; tick < 2000; ++tick) {
        input.now_s = tick * 0.02;
        input.pose_time_s = input.now_s;
        input.start_requested = tick == 0;
        input.offboard_stream_ready = prestream_started_s >= 0.0 && input.now_s - prestream_started_s >= 1.5;
        if (mission.state() == MissionState::kSearch) {
            input.target.boot_us = static_cast<std::uint64_t>(tick + 1) * 20000;
            input.target.present = input.target.has_range = true;
            input.target.conf = 0.9;
            input.target.class_id = 1;
            input.target.u_px = 320.0;
            input.target.v_px = 240.0;
            input.target.range_m = 3.0;
            input.target_time_s = input.now_s;
        } else if (mission.state() == MissionState::kStrike) {
            input.target.boot_us += 20000;
            input.target_time_s = input.now_s;
            input.strike_confirmed = input.now_s - state_entered_s >= 0.5;
        }
        const auto output = mission.tick(input);
        if (output.state_changed) {
            state_entered_s = input.now_s;
            std::printf("%6.2f s  %-12s reason=%s\n", input.now_s, toString(output.state), toString(output.reason));
        }
        // 以下为假飞控。真实接线时替换为异步命令执行器和实际反馈。
        if (output.request == FlightRequest::kOffboard) {
            input.offboard = true;
        } else if (output.request == FlightRequest::kArm) {
            input.armed = true;
        } else if (output.request == FlightRequest::kLand) {
            input.ins.p = mission.homeNed();
            input.landed = true;
            input.armed = input.offboard = false;
        }
        if (output.setpoint) {
            if (prestream_started_s < 0.0) {
                prestream_started_s = input.now_s;
            }
            // 合成的完美位置跟随，仅用于展示阶段切换。
            input.ins.p = {output.setpoint->x, output.setpoint->y, output.setpoint->z};
            if (output.state == MissionState::kTakeoff) {
                input.landed = false;
            }
        }
        if (output.state == MissionState::kCompleted) {
            std::printf("mission completed, strike_confirmed=%s\n", output.strike_confirmed ? "true" : "false");
            return output.strike_confirmed ? 0 : 1;
        }
        if (output.state == MissionState::kAborted) {
            return 1;
        }
    }
    std::fputs("mission example timed out\n", stderr);
    return 1;
}
