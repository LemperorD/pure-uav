#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "mission/mission_controller.hpp"

namespace {
using namespace pure_uav;
using namespace pure_uav::app::mission;

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void noControl(const MissionOutput& output) {
    check(!output.setpoint && !output.run_guidance && output.request == FlightRequest::kNone, "released control still produced a command");
}

struct Scenario {
    MissionController mission;
    MissionInput input;
    explicit Scenario(MissionConfig config = {}) : mission(config) {
        input.fcu_connected = input.preflight_ready = input.landed = true;
        input.ins.has_position = input.ins.has_attitude = true;
        input.ins.p = {2.0, 3.0, -0.4};
        input.guidance_ready = true;
    }
    MissionOutput tick(double dt = 0.02, bool refresh_pose = true) {
        input.now_s += dt;
        if (refresh_pose) {
            input.pose_time_s = input.now_s;
        }
        const auto result = mission.tick(input);
        check(!(result.setpoint && result.run_guidance), "two setpoint owners in one tick");
        return result;
    }
    void reach(MissionState state, unsigned max_ticks = 1000) {
        while (mission.state() != state && max_ticks-- > 0) {
            tick();
        }
        check(mission.state() == state, "failed to reach requested state");
    }
    void prepare() {
        input.start_requested = true;
        check(tick().state == MissionState::kPreflight, "start did not enter preflight");
        input.start_requested = false;
        check(tick().state == MissionState::kOffboard, "ready preflight did not capture home");
    }
    void takeoff() {
        prepare();
        input.offboard_stream_ready = true;
        check(tick().request == FlightRequest::kOffboard, "offboard request missing");
        input.offboard = true;
        check(tick().request == FlightRequest::kArm, "arm request missing");
        input.armed = true;
        check(tick().state == MissionState::kTakeoff, "armed did not begin takeoff");
    }
    void search() {
        takeoff();
        input.landed = false;
        input.ins.p.z -= 1.5;
        reach(MissionState::kSearch);
    }
    void target() {
        input.target.boot_us += 20000;
        input.target.present = input.target.has_range = true;
        input.target.conf = 0.9;
        input.target.class_id = 1;
        input.target.u_px = 320;
        input.target.v_px = 240;
        input.target.range_m = 2.0;
        input.target_time_s = input.now_s + 0.02;
    }
    void strike() {
        search();
        for (int frame = 0; frame < 3; ++frame) {
            target();
            tick();
        }
        check(mission.state() == MissionState::kStrike, "fresh confirmed target did not enter strike");
    }
};

void nominalMission() {
    Scenario s;
    noControl(s.tick());
    s.search();
    check(s.mission.hasHome() && std::abs(s.mission.homeNed().z + 0.4) < 1e-9, "home was not captured in local NED");
    check(std::abs(s.input.ins.p.z + 1.9) < 1e-9, "takeoff altitude used absolute zero");
    MissionOutput out;
    for (int frame = 0; frame < 3; ++frame) {
        s.target();
        out = s.tick();
    }
    check(out.run_guidance && out.reset_guidance, "strike did not acquire and reset guidance");
    out = s.tick();
    check(out.run_guidance && !out.reset_guidance, "guidance reset repeated");
    s.input.strike_confirmed = true;
    s.input.ins.p.x += 2.0;
    out = s.tick();
    check(out.state == MissionState::kReturnHome && out.strike_confirmed && out.setpoint, "strike confirmation did not return home");
    check(std::abs(out.setpoint->x - 2.0F) < 1e-6F && std::abs(out.setpoint->z + 1.9F) < 1e-6F, "return point was not above launch point");
    s.input.ins.p.x = 2.0;
    s.reach(MissionState::kLanding);
    noControl(s.tick());       // 降落请求已在进入该阶段时发出，重试间隔内不重复请求。
    s.input.offboard = false;  // 飞控自主降落可以退出 Offboard。
    s.input.landed = true;
    check(s.tick().state == MissionState::kLanding, "landing completed while still armed");
    s.input.armed = false;
    out = s.tick();
    check(out.state == MissionState::kCompleted && out.strike_confirmed, "landed/disarmed did not complete mission");
    noControl(out);
    s.input.start_requested = true;
    noControl(s.tick());
    check(s.mission.state() == MissionState::kCompleted, "terminal state restarted implicitly");
    s.mission.reset();
    check(!s.mission.hasHome() && s.mission.state() == MissionState::kIdle, "reset retained old mission");
}

void preflightAndRequests() {
    Scenario waiting;
    waiting.input.fcu_connected = false;
    waiting.input.start_requested = true;
    noControl(waiting.tick());
    waiting.reach(MissionState::kAborted);
    check(waiting.mission.reason() == MissionReason::kPreflightTimeout, "missing FCU bypassed preflight");

    Scenario stream;
    stream.prepare();
    check(stream.tick().request == FlightRequest::kNone, "offboard requested before real stream confirmation");
    stream.input.offboard_stream_ready = true;
    check(stream.tick().request == FlightRequest::kOffboard, "offboard request missing after stream ready");
    check(stream.tick().request == FlightRequest::kNone, "request retry was not throttled");
    check(stream.tick(1.01).request == FlightRequest::kOffboard, "request did not retry");

    Scenario settling;
    settling.takeoff();
    settling.input.landed = false;
    settling.input.ins.p.z -= 1.5;
    settling.tick();
    settling.input.ins.v.x = 0.5;
    settling.tick(0.1);
    settling.input.ins.v.x = 0.0;
    for (int i = 0; i < 20; ++i) {
        settling.tick();
    }
    check(settling.mission.state() == MissionState::kTakeoff, "position alone completed unsettled takeoff");
    settling.reach(MissionState::kSearch);
}

void targetConfirmationAndLoss() {
    Scenario s;
    s.search();
    s.target();
    s.tick();
    for (int repeat = 0; repeat < 5; ++repeat) {
        s.tick();
    }
    check(s.mission.state() == MissionState::kSearch, "duplicate frame counted toward confirmation");
    s.input.target.present = false;
    s.tick();
    s.target();
    s.tick();
    s.target();
    s.tick();
    check(s.mission.state() == MissionState::kSearch, "missing target failed to reset confirmation");
    s.target();
    check(s.tick().run_guidance, "three new frames did not activate guidance");
    s.input.ins.p.x += 0.8;
    s.input.target.present = false;
    auto out = s.tick();
    check(!out.run_guidance && out.setpoint && std::abs(out.setpoint->x - 2.8F) < 1e-6F, "lost target did not hold current position");
    s.reach(MissionState::kSearch);
    check(!s.tick().run_guidance, "target loss left guidance active");

    for (int invalid = 0; invalid < 6; ++invalid) {
        Scenario rejected;
        rejected.search();
        for (int frame = 0; frame < 5; ++frame) {
            rejected.target();
            switch (invalid) {
                case 0:
                    rejected.input.target_time_s = rejected.input.now_s - 1.0;
                    break;
                case 1:
                    rejected.input.target_time_s = rejected.input.now_s + 1.0;
                    break;
                case 2:
                    rejected.input.target.has_range = false;
                    break;
                case 3:
                    rejected.input.target.conf = 0.1;
                    break;
                case 4:
                    rejected.input.target.range_m = std::numeric_limits<double>::quiet_NaN();
                    break;
                case 5:
                    rejected.input.guidance_ready = false;
                    break;
            }
            check(!rejected.tick().run_guidance, "unusable target activated guidance");
        }
        check(rejected.mission.state() == MissionState::kSearch, "unusable target passed confirmation gate");
    }

    MissionConfig config;
    config.target_class = 0;
    Scenario wrong_class(config);
    wrong_class.search();
    for (int i = 0; i < 5; ++i) {
        wrong_class.target();
        wrong_class.tick();
    }
    check(wrong_class.mission.state() == MissionState::kSearch, "wrong target class accepted");
}

void timeoutsAndCancellation() {
    MissionConfig short_search;
    short_search.search_timeout_s = 0.1;
    Scenario search_timeout(short_search);
    search_timeout.search();
    search_timeout.reach(MissionState::kReturnHome);
    check(search_timeout.mission.reason() == MissionReason::kSearchTimeout, "search timeout reason lost");

    MissionConfig short_strike;
    short_strike.strike_timeout_s = 0.2;
    Scenario strike_timeout(short_strike);
    strike_timeout.strike();
    for (int i = 0; i < 12 && strike_timeout.mission.state() == MissionState::kStrike; ++i) {
        strike_timeout.target();
        strike_timeout.input.target.range_m = 0.01;  // 近距离不能当作撞击成功。
        strike_timeout.tick();
    }
    check(strike_timeout.mission.state() == MissionState::kReturnHome && strike_timeout.mission.reason() == MissionReason::kStrikeTimeout, "strike timeout did not return");
    check(!strike_timeout.tick().strike_confirmed, "proximity fabricated strike success");

    Scenario ground_cancel;
    ground_cancel.takeoff();
    ground_cancel.input.return_requested = true;
    const auto cancelled = ground_cancel.tick();
    check(cancelled.state == MissionState::kLanding && !cancelled.setpoint && cancelled.request == FlightRequest::kLand, "ground cancellation commanded a new ascent");

    Scenario airborne_cancel;
    airborne_cancel.search();
    airborne_cancel.input.return_requested = true;
    check(airborne_cancel.tick().state == MissionState::kReturnHome, "operator return was ignored");
    check(airborne_cancel.mission.reason() == MissionReason::kOperatorReturn, "operator return reason missing");

    MissionConfig short_total;
    short_total.mission_timeout_s = 0.8;
    Scenario total_timeout(short_total);
    total_timeout.search();
    total_timeout.reach(MissionState::kReturnHome);
    check(total_timeout.mission.reason() == MissionReason::kMissionTimeout, "total mission deadline ignored");

    MissionConfig short_return;
    short_return.return_timeout_s = 0.1;
    Scenario return_timeout(short_return);
    return_timeout.search();
    return_timeout.input.ins.p.x += 5.0;
    return_timeout.input.return_requested = true;
    return_timeout.tick();
    return_timeout.reach(MissionState::kLanding);
    check(return_timeout.mission.reason() == MissionReason::kStageTimeout, "return deadline did not delegate landing");
}

void faultsReleaseControl() {
    for (int fault = 0; fault < 8; ++fault) {
        Scenario s;
        s.search();
        MissionReason expected = MissionReason::kNone;
        bool fresh_pose = true;
        switch (fault) {
            case 0:
                s.input.manual_takeover = true;
                expected = MissionReason::kManualTakeover;
                break;
            case 1:
                s.input.fcu_connected = false;
                expected = MissionReason::kLinkLost;
                break;
            case 2:
                s.input.ins.p.x = std::numeric_limits<double>::quiet_NaN();
                expected = MissionReason::kPoseInvalid;
                break;
            case 3:
                s.input.offboard = false;
                expected = MissionReason::kOffboardLost;
                break;
            case 4:
                s.input.armed = false;
                expected = MissionReason::kUnexpectedDisarm;
                break;
            case 5:
                s.input.command_failed = true;
                expected = MissionReason::kCommandFailed;
                break;
            case 7:
                s.input.ins.p.x = std::numeric_limits<double>::max();
                expected = MissionReason::kPoseInvalid;
                break;
            case 6:
                s.input.pose_time_s = s.input.now_s - 1.0;
                fresh_pose = false;
                expected = MissionReason::kPoseInvalid;
                break;
        }
        const auto out = s.tick(0.02, fresh_pose);
        check(out.state == MissionState::kAborted && out.reason == expected, "fault did not abort with expected reason");
        noControl(out);
        noControl(s.tick());
    }
    Scenario backwards;
    backwards.search();
    backwards.input.now_s -= 1.0;
    const auto out = backwards.mission.tick(backwards.input);
    check(out.state == MissionState::kAborted && out.reason == MissionReason::kInvalidTime, "backwards clock accepted");
    noControl(out);

    MissionConfig invalid;
    invalid.request_retry_s = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try {
        MissionController controller(invalid);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "nonfinite config accepted");
}

}  // namespace

int main() {
    try {
        nominalMission();
        preflightAndRequests();
        targetConfirmationAndLoss();
        timeoutsAndCancellation();
        faultsReleaseControl();
        std::puts("PASS: mission flow, feedback gates, target freshness, timeouts and control release");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
