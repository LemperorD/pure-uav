#include <cstdio>
#include <stdexcept>

#include "pure_uav/vio_health.hpp"
void check(bool c, const char* why) {
    if (!c) throw std::runtime_error(why);
}
int main() {
    try {
        pure_uav::HealthConfig cfg;
        cfg.recover_after_s = 0.2;
        cfg.lost_after_s = 0.3;
        cfg.update_rate_window_s = 0.2;
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
        health.mark_implausible(6.1);
        health.reset();
        health.update(7, 120, true);
        check(health.publishable(), "new timebase failed to clear physical latch");
        // 纯视觉丢失仍允许已有的软恢复，不把所有 LOST 都锁死。
        for (int i = 1; i <= 30; ++i) health.update(7 + i * 0.02, 0, false);
        check(!health.publishable(), "visual loss not detected");
        for (int i = 31; i <= 100; ++i) health.update(7 + i * 0.02, 150, true);
        check(health.publishable(), "normal visual recovery regressed");
        // 首次 update 前的物理异常同样必须保持 LOST。
        health.reset();
        health.mark_implausible(10);
        health.update(10.1, 200, true);
        check(!health.publishable(), "first image bypassed physical latch");
        std::puts("PASS: physical latch and normal visual recovery");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
