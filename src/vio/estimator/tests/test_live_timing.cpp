#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "fcu_clock.hpp"
#include "live_timing.hpp"

void check(bool c, const char* why) {
    if (!c) {
        throw std::runtime_error(why);
    }
}
int main() {
    try {
        using pure_uav::FcuClock;
        FcuClock clock;
        pure_uav::ImuTimestampFilter filter;
        const int64_t host0 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        uint64_t previous = 0;
        uint64_t output = 0;
        int resets = 0;
        // 使用生产时钟和去抖器组成的实际处理链：正常 200Hz 后向前跳 120 秒。
        for (int i = 0; i < 300; ++i) {
            const uint64_t raw = 1000000ULL + (i * 5000ULL) + (i >= 100 ? 120000000ULL : 0);
            const auto result = clock.addSample(host0 + (i * 5000000LL), raw);
            if (result == FcuClock::AddResult::kTimebaseReset) {
                ++resets;
                filter.reset();
                previous = 0;
                check(!clock.valid() && clock.sampleCount() == 1, "clock retained old epoch");
            }
            check(result != FcuClock::AddResult::kOutOfOrder, "ordered IMU rejected");
            check(filter.filter(raw, 5000, output), "normal IMU rejected");
            if (previous != 0U) {
                check(output > previous && output - previous < 6000, "clock jump leaked into IMU propagation");
            }
            previous = output;
        }
        check(resets == 1, "forward clock jump was not detected exactly once");
        // 真实丢帧时重新锚定，而不是单帧丢弃后继续拖着旧网格追赶。
        filter.reset();
        check(filter.filter(1000000, 5000, output), "first sample");
        check(filter.filter(1040000, 5000, output) && output == 1040000, "gap not reanchored");
        check(filter.filter(1045000, 5000, output) && output == 1045000, "post-gap grid drifted");
        check(!filter.filter(1045000, 5000, output), "duplicate accepted");
        check(!filter.filter(1044000, 5000, output), "rollback accepted");
        // 匹配主机的调度停顿不是向前换时间基。
        FcuClock delayed;
        delayed.addSample(host0, 1000000);
        check(delayed.addSample(host0 + 2000000000LL, 3000000) == FcuClock::AddResult::kOk, "matching host/FC gap incorrectly changed epoch");
        check(delayed.addSample(host0 + 2005000000LL, 2995000) == FcuClock::AddResult::kOutOfOrder, "minor rollback not discarded");
        check(!pure_uav::imuCoversImage(0, 1000000, 0), "missing IMU covers image");
        check(!pure_uav::imuCoversImage(1005000, 1000000, 0.010), "positive calibration offset ignored");
        check(pure_uav::imuCoversImage(1015000, 1000000, 0.010), "valid offset coverage rejected");
        check(pure_uav::imuCoversImage(995000, 1000000, -0.010), "negative offset coverage rejected");
        check(!pure_uav::imuCoversImage(1000000, 1000000, NAN), "nonfinite offset accepted");
        check(pure_uav::odometryResetCounter(254) == 254 && pure_uav::odometryResetCounter(255) == 255 && pure_uav::odometryResetCounter(256) == 0 && pure_uav::odometryResetCounter(257) == 1, "reset_counter saturated");
        std::puts("PASS: forward epoch change, IMU gap, image coverage and reset wrap");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
