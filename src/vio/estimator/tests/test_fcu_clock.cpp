// FcuClock 的时间映射测试（不依赖硬件）。
//
// 模拟：主机单调时钟与飞控 IMU 时钟有一个固定偏移，IMU 到达有 0~2ms 的随机延迟；
// 检验 host→FCU 映射误差不超过几毫秒，且无样本时 valid() 为 false。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include "fcu_clock.hpp"

namespace {
int g_failures = 0;
}

int main() {
    using namespace pure_uav;
    FcuClock clock;

    if (clock.valid()) {
        std::fprintf(stderr, "FAIL 空时钟不应 valid\n");
        return 1;
    }

    // 用真实 CLOCK_MONOTONIC 构造最近 10 秒的样本（样本覆盖到"现在"附近）。
    using namespace std::chrono;
    const int64_t now_ns =
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    constexpr int kSamples = 2000;
    const int64_t sample_span_ns = (kSamples - 1) * 5000LL * 1000LL;  // 5 ms 间隔
    const int64_t base_host_ns = now_ns - sample_span_ns;
    const int64_t base_imu_us = 100000;
    const double true_offset_ns = static_cast<double>(base_host_ns - base_imu_us * 1000);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> jitter_us(0.0, 2000.0);

    for (int i = 0; i < kSamples; ++i) {
        const int64_t imu_us = base_imu_us + static_cast<int64_t>(i) * 5000;  // 200 Hz
        // 到达延迟：0~2 ms（单向延迟，所以 offset 只会偏大）
        const int64_t host_ns = imu_us * 1000 + static_cast<int64_t>(true_offset_ns) +
                                static_cast<int64_t>(jitter_us(rng) * 1000.0);
        const auto r = clock.addSample(host_ns, static_cast<uint64_t>(imu_us));
        if (r != FcuClock::AddResult::kOk) {
            std::fprintf(stderr, "FAIL 正常样本被判定为非 kOk\n");
            g_failures++;
        }
    }

    if (!clock.valid()) {
        std::fprintf(stderr, "FAIL 有足够样本后应 valid\n");
        g_failures++;
    }

    // 用"现在"之后 1ms 的主机时间反算 IMU 时间，误差应 < 3ms
    const int64_t host_probe = now_ns + 1000000;
    uint64_t mapped_us = 0;
    if (!clock.hostToFcuUs(host_probe, mapped_us)) {
        std::fprintf(stderr, "FAIL hostToFcuUs 返回 false\n");
        g_failures++;
    } else {
        const double expected_us = (static_cast<double>(host_probe) - true_offset_ns) / 1000.0;
        const double err_ms = std::fabs(static_cast<double>(mapped_us) - expected_us) / 1000.0;
        std::printf("映射误差: %.3f ms\n", err_ms);
        if (err_ms > 3.0) {
            std::fprintf(stderr, "FAIL 映射误差过大: %.3f ms\n", err_ms);
            g_failures++;
        }
    }

    // 偏移估计应接近真值 + 最小延迟（<2ms 偏差）
    const double off_err_ms =
        std::fabs(static_cast<double>(clock.offsetNs()) - true_offset_ns) / 1e6;
    std::printf("偏移估计误差: %.3f ms\n", off_err_ms);
    if (off_err_ms > 2.0) {
        std::fprintf(stderr, "FAIL 偏移估计误差过大: %.3f ms\n", off_err_ms);
        g_failures++;
    }

    //---------------------------------------------------------------------------
    // 用例 2：轻微乱序（5 ms 回退）只丢帧，不能触发时间基重置
    //---------------------------------------------------------------------------
    {
        const size_t before = clock.sampleCount();
        const uint64_t last_us = clock.lastImuTimestampUs();
        const auto r = clock.addSample(now_ns, static_cast<uint64_t>(last_us - 5000));
        if (r != FcuClock::AddResult::kOutOfOrder) {
            std::fprintf(stderr, "FAIL 5ms 回退应判 kOutOfOrder\n");
            g_failures++;
        }
        if (clock.sampleCount() != before) {
            std::fprintf(stderr, "FAIL 乱序帧不应进入偏移估计\n");
            g_failures++;
        }
        if (!clock.valid()) {
            std::fprintf(stderr, "FAIL 轻微乱序不应让时钟失效\n");
            g_failures++;
        }
        if (clock.timebaseResetCount() != 0) {
            std::fprintf(stderr, "FAIL 轻微乱序不应计为时间基重置\n");
            g_failures++;
        }
    }

    //---------------------------------------------------------------------------
    // 用例 3：飞控重启。模拟"运行到约 303 s 后重启，boot 时间回到 1.075 s"。
    // 旧实现会继续用旧 offset，映射出约 303 s；现在必须清空旧样本并判 kTimebaseReset。
    //---------------------------------------------------------------------------
    {
        const uint64_t fcu_before_reboot_us = clock.lastImuTimestampUs();
        const uint64_t reboot_boot_us = 1075000;  // 重启后 1.075 s
        const int64_t host_at_reboot_ns = now_ns + 10000000;  // 重启帧此时到达
        const auto r = clock.addSample(host_at_reboot_ns, reboot_boot_us);
        if (r != FcuClock::AddResult::kTimebaseReset) {
            std::fprintf(stderr,
                         "FAIL 飞控重启（%llu us → %llu us）未判 kTimebaseReset\n",
                         static_cast<unsigned long long>(fcu_before_reboot_us),
                         static_cast<unsigned long long>(reboot_boot_us));
            g_failures++;
        }
        if (clock.valid()) {
            std::fprintf(stderr, "FAIL 时间基重置后 valid() 必须为 false（要重新累积样本）\n");
            g_failures++;
        }
        if (clock.sampleCount() != 1) {
            std::fprintf(stderr, "FAIL 时间基重置后应只剩新基的 1 帧，实际 %zu\n",
                         clock.sampleCount());
            g_failures++;
        }
        if (clock.timebaseResetCount() != 1) {
            std::fprintf(stderr, "FAIL timebaseResetCount 应为 1\n");
            g_failures++;
        }

        // 新时间基重新累积 30 帧（≈150 ms），映射必须落在新基附近，而不是旧的 303 s。
        for (int i = 1; i <= 30; ++i) {
            const int64_t imu_us = static_cast<int64_t>(reboot_boot_us) + i * 5000;
            const int64_t host_ns = host_at_reboot_ns + i * 5000LL * 1000LL;
            const auto rr = clock.addSample(host_ns, static_cast<uint64_t>(imu_us));
            if (rr != FcuClock::AddResult::kOk) {
                std::fprintf(stderr, "FAIL 新时间基样本被误判（i=%d）\n", i);
                g_failures++;
            }
        }
        if (!clock.valid()) {
            std::fprintf(stderr, "FAIL 新时间基样本足够后应恢复 valid\n");
            g_failures++;
        }
        uint64_t mapped = 0;
        const int64_t probe_host_ns = host_at_reboot_ns + 30 * 5000LL * 1000LL;
        if (!clock.hostToFcuUs(probe_host_ns, mapped)) {
            std::fprintf(stderr, "FAIL 新时间基 hostToFcuUs 返回 false\n");
            g_failures++;
        } else {
            const double mapped_s = static_cast<double>(mapped) * 1e-6;
            std::printf("重启后映射: %.3f s（旧时间基约 303 s）\n", mapped_s);
            if (mapped_s > 10.0) {
                std::fprintf(stderr, "FAIL 重启后映射仍落在旧时间基: %.3f s\n", mapped_s);
                g_failures++;
            }
        }
    }

    if (g_failures == 0) {
        std::printf("PASS: FcuClock 时间映射测试通过（含飞控重启/乱序）\n");
        return 0;
    }
    return 1;
}
