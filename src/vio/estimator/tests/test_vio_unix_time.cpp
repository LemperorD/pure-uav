// Online VIO time conversion without a camera, flight controller or dataset.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "fcu_clock.hpp"
#include "live_timing.hpp"

namespace {
int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
uint64_t unixNowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}
void check(bool condition, const char* reason) {
    if (!condition) {
        throw std::runtime_error(reason);
    }
}
void populate(pure_uav::FcuClock& clock, int64_t host_ns, uint64_t fcu_us, int samples = 20) {
    for (int i = 0; i < samples; ++i) {
        check(clock.addSample(host_ns + (i * 5000000LL), fcu_us + (i * 5000ULL)) == pure_uav::FcuClock::AddResult::kOk, "ordered clock sample rejected");
    }
}
void checkHostEpoch(pure_uav::FcuClock& clock, int64_t host_ns) {
    uint64_t fcu_us = 0;
    uint64_t unix_us = 0;
    check(clock.hostToFcuUs(host_ns, fcu_us) && clock.fcuToUnixUs(fcu_us, unix_us), "host to Unix conversion failed");
    const uint64_t expected = unixNowUs() - static_cast<uint64_t>((steadyNowNs() - host_ns) / 1000);
    const uint64_t error = unix_us > expected ? unix_us - expected : expected - unix_us;
    check(error < 10000, "Unix epoch does not match host system_clock");
}
}  // namespace

int main() {
    try {
        using pure_uav::FcuClock;
        FcuClock clock;
        uint64_t output = 42;
        check(!clock.fcuToUnixUs(1000000, output) && output == 42, "unready clock leaked a raw timestamp");
        const int64_t host0 = steadyNowNs() - 500000000LL;
        constexpr uint64_t kBoot0 = 5000000;
        populate(clock, host0, kBoot0, 9);
        check(!clock.fcuToUnixUs(kBoot0, output), "insufficient clock samples accepted");
        populate(clock, host0 + 45000000LL, kBoot0 + 45000, 11);
        checkHostEpoch(clock, host0);
        uint64_t anchor = 0;
        check(clock.fcuToUnixUs(kBoot0, anchor), "boot to Unix conversion failed");
        check(clock.fcuToUnixUs(kBoot0 + 1, output) && output == anchor + 1, "one microsecond lost during epoch conversion");
        check(clock.fcuToUnixUs(kBoot0 + 5037, output) && output == anchor + 5037, "sampling interval changed during epoch conversion");
        check(!clock.fcuToUnixUs(0, output), "zero FCU timestamp accepted");
        check(!clock.fcuToUnixUs(std::numeric_limits<uint64_t>::max(), output), "out-of-range FCU timestamp accepted");

        // Later offset estimates and IMU jitter must not change the locked epoch.
        const int64_t old_offset = clock.offsetNs();
        populate(clock, host0 + 80000000LL, kBoot0 + 100000, 40);
        check(clock.offsetNs() != old_offset, "test did not exercise offset refinement");
        check(clock.fcuToUnixUs(kBoot0, output) && output == anchor, "offset refinement moved the Unix epoch");
        pure_uav::ImuTimestampFilter filter;
        uint64_t previous_fcu = 0;
        uint64_t previous_unix = 0;
        for (int i = 0; i < 200; ++i) {
            const uint64_t raw = kBoot0 + (i * 5000ULL) + (i % 2 == 0 ? 0 : 300);
            uint64_t filtered = 0;
            check(filter.filter(raw, 5000, filtered), "normal IMU jitter was rejected");
            check(clock.fcuToUnixUs(filtered, output), "filtered IMU conversion failed");
            if (previous_fcu != 0) {
                check(output - previous_unix == filtered - previous_fcu, "Unix conversion changed filtered IMU intervals");
            }
            previous_fcu = filtered;
            previous_unix = output;
        }
        uint64_t color_fcu = 0;
        uint64_t color_unix = 0;
        uint64_t depth_unix = 0;
        check(clock.hostToFcuUs(host0 + 200000000LL, color_fcu), "camera mapping failed");
        check(clock.fcuToUnixUs(color_fcu, color_unix) && clock.fcuToUnixUs(color_fcu - 1234, depth_unix), "image/depth conversion failed");
        check(color_unix - depth_unix == 1234, "depth/color sampling offset changed");
        uint64_t imu_unix = 0;
        check(clock.fcuToUnixUs(color_fcu + 5000, imu_unix), "image coverage IMU conversion failed");
        check(pure_uav::imuCoversImage(imu_unix, color_unix, 0.004), "aligned Unix image is not covered by IMU");
        check(!pure_uav::imuCoversImage(imu_unix, color_unix, 0.006), "camera calibration offset ignored");

        clock.reset();
        check(!clock.fcuToUnixUs(kBoot0, output), "manual reset retained the old epoch");
        const int64_t host1 = steadyNowNs() - 100000000LL;
        populate(clock, host1, kBoot0);
        checkHostEpoch(clock, host1);
        check(clock.addSample(steadyNowNs(), 1000) == FcuClock::AddResult::kTimebaseReset, "reboot was not detected");
        check(!clock.fcuToUnixUs(1000, output), "reboot retained the old Unix mapping");
        const int64_t host2 = steadyNowNs();
        populate(clock, host2 + 5000000LL, 6000);
        checkHostEpoch(clock, host2);

        FcuClock stale;
        populate(stale, steadyNowNs() - 3000000000LL, kBoot0);
        check(!stale.fcuToUnixUs(kBoot0, output), "stale clock still converted timestamps");

        // HIGHRES_IMU may already carry an epoch; map it to the same host source.
        FcuClock epoch_input;
        const int64_t host3 = steadyNowNs() - 100000000LL;
        const uint64_t raw_epoch = unixNowUs() - 100000;
        populate(epoch_input, host3, raw_epoch);
        checkHostEpoch(epoch_input, host3);
        check(epoch_input.fcuToUnixUs(raw_epoch, anchor) && epoch_input.fcuToUnixUs(raw_epoch + 1, output) && output == anchor + 1, "epoch IMU input lost microsecond precision");
        check(!epoch_input.hostToFcuUs(std::numeric_limits<int64_t>::max(), output), "nanosecond subtraction overflow accepted");
        check(!epoch_input.hostToFcuUs(-1, output), "negative host time accepted");
        check(epoch_input.addSample(std::numeric_limits<int64_t>::min(), raw_epoch) == FcuClock::AddResult::kOutOfOrder, "invalid host sample accepted");

        FcuClock large_input;
        constexpr uint64_t kLarge = 9007199254740993ULL;  // Above the exact integer range of double.
        populate(large_input, steadyNowNs() - 100000000LL, kLarge);
        check(large_input.fcuToUnixUs(kLarge, anchor) && large_input.fcuToUnixUs(kLarge + 1, output) && output == anchor + 1, "large FCU timestamp lost integer precision");
        check(!large_input.fcuToUnixUs(1, output), "negative Unix result wrapped to uint64_t");
        std::puts("PASS: VIO Unix epoch, microseconds, fixed offset, IMU/image/depth alignment, reset, stale clock and ranges");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
