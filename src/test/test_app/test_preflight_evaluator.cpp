#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "glue/preflight_evaluator.hpp"

using namespace pure_uav;
using namespace pure_uav::app;

namespace {
int checks = 0;

void expect(bool value, const char* reason) {
    ++checks;
    if (!value) {
        throw std::runtime_error(reason);
    }
}

PreflightInput validInput() {
    PreflightInput input;
    input.connected = true;
    input.target_locked = true;
    input.autopilot_px4 = true;
    input.session_stable = true;
    input.armed = false;
    input.landed = true;
    input.mode = px4_mode::AUTO_LOITER;
    input.heartbeat_age_ms = 100;
    input.attitude_seen = true;
    input.attitude_age_ms = 50;
    input.attitude.q[0] = 1.0F;
    input.attitude.q[1] = 0.0F;
    input.attitude.q[2] = 0.0F;
    input.attitude.q[3] = 0.0F;
    input.position_seen = true;
    input.position_age_ms = 50;
    input.battery_seen = true;
    input.battery.voltage = 16.0F;
    input.battery.remaining = 80;
    input.system_time_seen = true;
    input.system_time_age_ms = 100;
    return input;
}
}  // namespace

int main() {
    try {
        const PreflightConfig config;
        const PreflightEvaluator evaluator(config);
        expect(evaluator.evaluate(validInput()).ok, "valid preflight input should pass");

        PreflightInput input = validInput();
        input.connected = false;
        expect(!evaluator.evaluate(input).ok, "disconnected input accepted");
        input = validInput();
        input.target_locked = false;
        expect(!evaluator.evaluate(input).ok, "unlocked target accepted");
        input = validInput();
        input.autopilot_px4 = false;
        expect(!evaluator.evaluate(input).ok, "non-PX4 accepted");
        input = validInput();
        input.session_stable = false;
        expect(!evaluator.evaluate(input).ok, "unstable session accepted");
        input = validInput();
        input.heartbeat_age_ms = config.heartbeat_max_age_ms + 1;
        expect(!evaluator.evaluate(input).ok, "stale heartbeat accepted");
        input = validInput();
        input.armed = true;
        expect(!evaluator.evaluate(input).ok, "armed input accepted");
        input = validInput();
        input.landed = false;
        expect(!evaluator.evaluate(input).ok, "airborne state accepted");
        input = validInput();
        input.mode = px4_mode::OFFBOARD;
        expect(!evaluator.evaluate(input).ok, "OFFBOARD mode accepted");
        input = validInput();
        input.attitude_seen = false;
        expect(!evaluator.evaluate(input).ok, "missing attitude accepted");
        input = validInput();
        input.attitude_age_ms = config.attitude_max_age_ms + 1;
        expect(!evaluator.evaluate(input).ok, "stale attitude accepted");
        input = validInput();
        input.attitude.q[0] = std::nanf("");
        expect(!evaluator.evaluate(input).ok, "non-finite attitude accepted");
        input = validInput();
        input.attitude.q[0] = 2.0F;
        expect(!evaluator.evaluate(input).ok, "invalid quaternion accepted");
        input = validInput();
        input.attitude.roll = static_cast<float>(config.max_tilt_deg * 3.14159265358979323846 / 180.0 + 0.1);
        expect(!evaluator.evaluate(input).ok, "excessive tilt accepted");
        input = validInput();
        input.position_seen = false;
        expect(!evaluator.evaluate(input).ok, "missing local position accepted");
        input = validInput();
        input.position_age_ms = config.local_position_max_age_ms + 1;
        expect(!evaluator.evaluate(input).ok, "stale local position accepted");
        input = validInput();
        input.position.vz = static_cast<float>(config.max_vertical_speed_mps + 0.1);
        expect(!evaluator.evaluate(input).ok, "excessive vertical speed accepted");
        input = validInput();
        input.system_time_seen = false;
        expect(!evaluator.evaluate(input).ok, "missing system time accepted");
        input = validInput();
        input.system_time_age_ms = config.system_time_max_age_ms + 1;
        expect(!evaluator.evaluate(input).ok, "stale system time accepted");
        input = validInput();
        input.battery_seen = false;
        expect(!evaluator.evaluate(input).ok, "missing battery accepted");
        input = validInput();
        input.battery.voltage = 0.0F;
        expect(!evaluator.evaluate(input).ok, "invalid battery voltage accepted");
        input = validInput();
        input.battery.remaining = static_cast<int8_t>(config.min_battery_percent - 1);
        expect(!evaluator.evaluate(input).ok, "low battery accepted");

        std::printf("PASS: %d preflight evaluator checks\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
