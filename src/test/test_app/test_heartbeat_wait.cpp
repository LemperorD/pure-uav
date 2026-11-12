#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "glue/comm_imu_source.hpp"
using namespace pure_uav;
using namespace pure_uav::app;
void check(bool value, const char* reason) {
    if (!value) {
        throw std::runtime_error(reason);
    }
}
int main() {
    try {
        Communicator comm;  // deliberately never start it: no serial or camera is opened
        std::atomic<bool> stop{false};
        auto before = std::chrono::steady_clock::now();
        check(waitForHeartbeat(comm, stop, 0.05) == HeartbeatWaitResult::timeout, "heartbeat did not time out");
        auto elapsed = std::chrono::steady_clock::now() - before;
        check(elapsed >= std::chrono::milliseconds(50) && elapsed < std::chrono::milliseconds(300), "timeout is unbounded");
        stop.store(true);
        before = std::chrono::steady_clock::now();
        check(waitForHeartbeat(comm, stop, 5) == HeartbeatWaitResult::stopped, "preexisting stop was ignored");
        check(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(100), "immediate stop blocked");
        stop.store(false);
        std::thread interrupt([&] {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            stop.store(true);
        });
        before = std::chrono::steady_clock::now();
        const auto result = waitForHeartbeat(comm, stop, 5);
        interrupt.join();
        check(result == HeartbeatWaitResult::stopped, "stop during heartbeat wait was ignored");
        check(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(300), "stop during heartbeat wait blocked");
        std::puts("PASS: actual main heartbeat wait timeout and cancellation; no devices opened");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
