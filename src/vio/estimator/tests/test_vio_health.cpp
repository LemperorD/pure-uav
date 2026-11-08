#include <cstdio>
#include <stdexcept>
#include "pure_uav/vio_health.hpp"
void check(bool c, const char* why) { if (!c) throw std::runtime_error(why); }

void checkReasons() {
    using pure_uav::HealthReason;
    pure_uav::HealthConfig cfg;
    cfg.lost_after_s = 0.3;
    cfg.degrade_after_s = 0.1;
    cfg.update_rate_window_s = 0.2;
    cfg.min_update_rate = 0.0; // Isolate the timeout/feature/depth/tilt paths.
    cfg.deperr_lost_s = 0.1;
    cfg.tilt_lost_s = 0.1;
    pure_uav::HealthMonitor health(cfg);
    health.update(0, 120, true);
    check(health.report(0).reason == HealthReason::NONE, "healthy reason");
    check(health.report(0).update_rate < 0, "warming-up rate must be unavailable");
    for (int i = 1; i <= 20; ++i) health.update(i * .02, 120, false);
    check(!health.publishable() && health.report(.4).reason == HealthReason::VISUAL_TIMEOUT,
          "visual timeout diagnostic");
    health.reset(); health.update(0, 120, true);
    for (int i = 1; i <= 20; ++i) health.update(i * .02, 0, true);
    check(!health.publishable() && health.report(.4).reason == HealthReason::FEATURES,
          "feature starvation diagnostic");
    health.reset(); health.update(0, 120, true);
    for (int i = 1; i <= 12; ++i) health.update(i * .02, 120, true, 20.0, 0.0);
    check(!health.publishable() && health.report(.24).reason == HealthReason::DEPTH,
          "depth diagnostic");
    health.reset(); health.update(0, 120, true);
    for (int i = 1; i <= 12; ++i) health.update(i * .02, 120, true, 1.0, 10.0);
    check(!health.publishable() && health.report(.24).reason == HealthReason::TILT,
          "tilt diagnostic");
    cfg.min_update_rate = .15; cfg.lost_after_s = 100;
    health.configure(cfg); health.update(0, 120, true);
    for (int i = 1; i <= 20; ++i) health.update(i * .02, 120, false);
    check(!health.publishable() && health.report(.4).reason == HealthReason::UPDATE_RATE,
          "low update rate diagnostic");
    check(health.report(.4).update_rate == 0, "rate should reflect failed visual updates");
    health.mark_implausible(.42);
    health.update(.44, 200, true);
    check(!health.publishable() && health.report(.44).reason == HealthReason::PHYSICAL,
          "physical fault remains latched and identified");
    health.notify_recovered();
    check(health.report(.44).reason == HealthReason::NONE && health.report(.44).update_rate < 0,
          "rebuild must clear diagnostics and rate history");
    cfg.enabled = false; health.configure(cfg); health.update(1, 0, false, 100, 90);
    check(health.publishable() && health.report(1).reason == HealthReason::NONE,
          "disabled health behavior changed");
}

int main() {
    try {
        checkReasons();
        pure_uav::HealthConfig cfg;
        cfg.recover_after_s = 0.2; cfg.lost_after_s = 0.3; cfg.update_rate_window_s = 0.2;
        pure_uav::HealthMonitor health(cfg);
        health.update(0, 120, true);
        check(health.publishable(), "initial healthy state");
        health.mark_implausible(0.1);
        for (int i = 6; i <= 300; ++i) {
            health.update(i * 0.02, 200, true, 1.0, 0.0);
            check(!health.publishable(), "visual evidence cleared physical fault without rebuild");
            if (i % 10 == 0) health.mark_implausible(i * 0.02);
        }
        check(health.report(6).lost_count == 1, "repeated fault changed LOST transition count");
        health.notify_recovered();
        check(health.publishable(), "reconstruction failed to clear physical latch");
        health.update(6.02, 120, false);
        check(health.publishable(), "reconstruction inherited old loss timer");
        health.mark_implausible(6.1); health.reset(); health.update(7, 120, true);
        check(health.publishable(), "new timebase failed to clear physical latch");
        // 纯视觉丢失仍允许已有的软恢复，不把所有 LOST 都锁死。
        for (int i = 1; i <= 30; ++i) health.update(7 + i * 0.02, 0, false);
        check(!health.publishable(), "visual loss not detected");
        for (int i = 31; i <= 100; ++i) health.update(7 + i * 0.02, 150, true);
        check(health.publishable(), "normal visual recovery regressed");
        // 首次 update 前的物理异常同样必须保持 LOST。
        health.reset(); health.mark_implausible(10); health.update(10.1, 200, true);
        check(!health.publishable(), "first image bypassed physical latch");
        std::puts("PASS: physical latch and normal visual recovery"); return 0;
    } catch (const std::exception& e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
