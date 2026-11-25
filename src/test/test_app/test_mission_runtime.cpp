#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "glue/mission_ports.hpp"
#include "mission/mission_runtime.hpp"

namespace {
using namespace pure_uav;
using namespace pure_uav::app;
using namespace pure_uav::app::mission;
void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
struct Flight final : MissionFlightPort {
    FlightFeedback value;
    bool send_ok = true, request_ok = true, follow = true;
    unsigned commands = 0, requests = 0;
    std::vector<char> ordering;
    Flight() {
        value.supported = true;
        value.session = 7;
        auto& input = value.input;
        input.fcu_connected = input.preflight_ready = input.landed = true;
        input.ins.has_position = input.ins.has_attitude = true;
        input.ins.p = {2, 3, -0.4};
    }
    FlightFeedback feedback(double now_s) override {
        value.input.now_s = value.input.pose_time_s = now_s;
        value.input.ins.boot_us += 20000;
        return value;
    }
    bool send(const GuidanceCmd& command, std::uint64_t session) override {
        check(session == value.session, "wrong send session");
        ++commands;
        ordering.push_back('S');
        if (follow && value.input.armed && command.mode == GuidanceMode::POSITION) {
            value.input.ins.p = {command.x, command.y, command.z};
            value.input.landed = false;
        }
        return send_ok;
    }
    bool request(FlightRequest request, std::uint64_t session) override {
        check(session == value.session, "wrong request session");
        ++requests;
        if (request == FlightRequest::kOffboard || request == FlightRequest::kArm) {
            check(!ordering.empty() && ordering.back() == 'S', "mode/arm request preceded this tick's setpoint");
        }
        ordering.push_back('R');
        if (request == FlightRequest::kOffboard) {
            value.input.offboard = true;
        }
        if (request == FlightRequest::kArm) {
            value.input.armed = true;
        }
        if (request == FlightRequest::kLand) {
            value.input.landed = true;
            value.input.armed = value.input.offboard = false;
        }
        return request_ok;
    }
};
struct Guidance final : MissionGuidancePort {
    bool healthy = true, reset_ok = true;
    unsigned resets = 0, steps = 0;
    bool ready() const override {
        return true;
    }
    bool reset() override {
        ++resets;
        return reset_ok;
    }
    std::optional<GuidanceCmd> step(const MissionInput& /*input*/) override {
        ++steps;
        if (!healthy) {
            return std::nullopt;
        }
        return GuidanceCmd{};
    }
};
struct Scenario {
    Flight flight;
    Guidance guidance;
    RuntimeConfig config;
    MissionRuntime runtime;
    RuntimeSignals signals;
    explicit Scenario(bool send = true) : config(configuration(send)), runtime(config, flight, &guidance) {
        signals.navigation_ready = signals.start_requested = true;
        signals.navigation_generation = 4;
    }
    static RuntimeConfig configuration(bool send) {
        RuntimeConfig c;
        c.send_enabled = send;
        c.prestream_s = 0.15;
        c.max_setpoint_gap_s = 0.05;
        c.mission.settle_time_s = 0.05;
        c.mission.target_confirm_frames = 2;
        return c;
    }
    RuntimeStatus tick(double dt = 0.02) {
        signals.now_s += dt;
        if (signals.target.present) {
            signals.target.boot_us += 20000;
            signals.target_time_s = signals.now_s;
        }
        return runtime.tick(signals);
    }
    void reach(MissionState state, unsigned max_ticks = 100) {
        while (runtime.status().mission.state != state && max_ticks-- > 0) {
            tick();
        }
        check(runtime.status().mission.state == state, "runtime failed to reach requested state");
    }
    void strike() {
        reach(MissionState::kSearch);
        signals.target.present = signals.target.has_range = true;
        signals.target.conf = 0.9;
        signals.target.range_m = 2;
        reach(MissionState::kStrike);
    }
};

void nominal() {
    Scenario s;
    s.strike();
    check(s.guidance.resets == 1 && s.guidance.steps == 1, "guidance entry not reset once");
    const auto sent = s.flight.commands;
    s.tick();
    check(s.flight.commands == sent + 1 && s.guidance.steps == 2, "guidance did not use the single output owner");
    s.signals.strike_confirmed = true;
    check(s.tick().mission.state == MissionState::kReturnHome, "strike event did not reach runtime");
    const auto steps = s.guidance.steps;
    s.reach(MissionState::kCompleted);
    check(s.guidance.steps == steps && s.runtime.status().mission.strike_confirmed, "guidance continued after return or confirmation lost");
    const auto count = s.flight.commands + s.flight.requests;
    s.tick();
    check(s.flight.commands + s.flight.requests == count, "terminal runtime sent more commands");
}
void dryRunAndStream() {
    Scenario dry(false);
    for (int i = 0; i < 50; ++i) {
        dry.tick();
    }
    check(dry.flight.commands == 0 && dry.flight.requests == 0 && dry.runtime.status().would_send > 0, "no-send emitted control");
    check(dry.runtime.status().mission.state == MissionState::kOffboard, "dry run faked offboard prestream success");
    Scenario gap;
    gap.reach(MissionState::kOffboard);
    for (int i = 0; i < 4; ++i) {
        gap.tick();
    }
    gap.tick(0.1);
    check(gap.flight.requests == 0, "send gap did not reset prestream");
    for (int i = 0; i < 6; ++i) {
        gap.tick();
    }
    check(gap.flight.requests == 0, "prestream counted time before gap");
    gap.reach(MissionState::kSearch);
}
void failureAndShutdown() {
    for (int fault = 0; fault < 6; ++fault) {
        Scenario s;
        s.reach(MissionState::kSearch);
        MissionReason reason;
        if (fault == 0) {
            s.flight.send_ok = false;
            reason = MissionReason::kCommandFailed;
        } else if (fault == 1) {
            ++s.flight.value.session;
            reason = MissionReason::kSessionChanged;
        } else if (fault == 2) {
            ++s.signals.navigation_generation;
            reason = MissionReason::kNavigationReset;
        } else if (fault == 3) {
            s.signals.manual_takeover = true;
            reason = MissionReason::kManualTakeover;
        } else if (fault == 4) {
            s.flight.value.input.offboard = false;
            reason = MissionReason::kOffboardLost;
        } else {
            s.signals.navigation_ready = false;
            reason = MissionReason::kPoseInvalid;
        }
        const auto before = s.flight.commands + s.flight.requests;
        const auto status = s.tick();
        check(status.mission.state == MissionState::kAborted && status.mission.reason == reason, "runtime fault reason lost");
        const auto count = s.flight.commands + s.flight.requests;
        if (fault != 0) {
            check(before == count, "fault sent control before releasing");
        }
        s.tick();
        check(s.flight.commands + s.flight.requests == count, "failure left queued work");
    }
    Scenario failure;
    failure.flight.request_ok = false;
    failure.reach(MissionState::kAborted);
    check(failure.runtime.status().mission.reason == MissionReason::kCommandFailed, "request failure ignored");
    Scenario stopped;
    stopped.reach(MissionState::kArming);
    const auto count = stopped.flight.commands + stopped.flight.requests;
    stopped.runtime.stop();
    stopped.tick();
    check(stopped.flight.commands + stopped.flight.requests == count, "stop left an old arm queued");
    Scenario guidance_failure;
    guidance_failure.guidance.reset_ok = false;
    guidance_failure.signals.target.present = guidance_failure.signals.target.has_range = true;
    guidance_failure.signals.target.conf = 0.9;
    guidance_failure.signals.target.range_m = 2;
    guidance_failure.reach(MissionState::kAborted);
    check(guidance_failure.guidance.steps == 0, "failed guidance reset still ran");
}
void realGuidance() {
    GuidanceAppParams params;
    params.loop.cam.fx = params.loop.cam.fy = 500;
    params.loop.cam.cx = 320;
    params.loop.cam.cy = 240;
    params.loop.cam.R_body_cam[0] = 0;
    params.loop.cam.R_body_cam[1] = 0;
    params.loop.cam.R_body_cam[2] = 1;
    params.loop.cam.R_body_cam[3] = 1;
    params.loop.cam.R_body_cam[4] = 0;
    params.loop.cam.R_body_cam[5] = 0;
    params.loop.cam.R_body_cam[6] = 0;
    params.loop.cam.R_body_cam[7] = 1;
    params.loop.cam.R_body_cam[8] = 0;
    MissionGuidanceRunner runner(params);
    check(runner.ready(), "configured guidance was not ready");
    MissionInput input;
    input.ins.has_position = input.ins.has_attitude = true;
    input.ins.boot_us = 1000000;
    input.target.boot_us = input.ins.boot_us;
    input.target.present = input.target.has_range = true;
    input.target.u_px = 320;
    input.target.v_px = 240;
    input.target.range_m = 5;
    input.target.conf = 0.9;
    check(runner.step(input).has_value(), "real guidance adapter failed a valid frame");
    check(runner.reset(), "real guidance could not reset");
    input.ins.boot_us = input.target.boot_us = 500000;  // 新实例能接受更早的首帧。
    check(runner.step(input).has_value(), "guidance reset retained old sample time");
}
}  // namespace

int main() {
    try {
        nominal();
        dryRunAndStream();
        failureAndShutdown();
        realGuidance();
        std::puts("PASS: main/mission runtime, single command owner, send gating, feedback, resets and shutdown");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
