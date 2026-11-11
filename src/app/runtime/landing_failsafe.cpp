#include "runtime/landing_failsafe.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include "communicator.hpp"

namespace pure_uav::app {
namespace {
using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
}  // namespace

bool runLandingFailsafe(Communicator& comm, double timeout_s, double request_interval_s, const std::atomic<bool>& interrupted, const std::function<bool()>& keepalive) {
    const double timeout = timeout_s > 0.0 ? timeout_s : 20.0;
    const double interval = request_interval_s > 0.0 ? request_interval_s : 1.0;
    const auto start = Clock::now();
    auto next_request = start;
    std::printf("[FAILSAFE] 请求飞控降落，timeout=%.1fs retry=%.1fs\n", timeout, interval);

    while (secondsSince(start) < timeout) {
        if (interrupted.load(std::memory_order_relaxed)) {
            std::fprintf(stderr, "[FAILSAFE] 收到中断，停止降落等待\n");
            return false;
        }
        if (keepalive) {
            (void)keepalive();
        }

        LandedState landed;
        const bool have_landed = comm.getLandedState(landed);
        if (have_landed && landed.state == 1 && !comm.isArmed()) {
            std::printf("[FAILSAFE] 飞控已确认落地并上锁\n");
            return true;
        }

        const auto now = Clock::now();
        if (now >= next_request) {
            const auto session = comm.reconnectCount();
            if (!comm.requestLandForSession(session)) {
                std::fprintf(stderr, "[FAILSAFE] NAV_LAND 发送失败 session=%llu\n", static_cast<unsigned long long>(session));
            }
            next_request = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(interval));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::fprintf(stderr, "[FAILSAFE] 降落等待超时 %.1fs\n", timeout);
    return false;
}

}  // namespace pure_uav::app
