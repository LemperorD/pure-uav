// VIO 主循环与检测进程监督并行时的退出和资源回收，不打开实际设备。
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "detection_supervisor.hpp"

namespace {
using namespace pure_uav::app;
using Clock = std::chrono::steady_clock;

void check(bool value, const char* reason) {
    if (!value) {
        throw std::runtime_error(reason);
    }
}
DetectionProcessOptions options(const std::string& binary) {
    DetectionProcessOptions result;
    result.executable = binary;
    result.poll_interval_ms = 10;
    result.restart_delay_ms = 10;
    result.stop_timeout_ms = 500;
    result.max_restarts = 1;
    return result;
}
SuperviseResult waitForExit(DetectionSupervisor& process) {
    SuperviseResult result;
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (!process.finished(result)) {
        check(Clock::now() < deadline, "supervision did not finish");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return result;
}
void normalExitAndRestartLimit() {
    std::string error;
    DetectionSupervisor normal(options("/bin/true"));
    check(normal.start(&error), "normal process start failed");
    check(waitForExit(normal).status == SuperviseStatus::exited, "normal exit became an error");
    normal.stop();
    auto config = options("/bin/false");
    DetectionSupervisor failing(config);
    check(failing.start(&error), "failing process could not start");
    const auto result = waitForExit(failing);
    check(result.status == SuperviseStatus::restart_limit, "crashing detector was not bounded");
    check(result.restarts == 1 && result.last_exit_code == 1, "restart/exit status was lost");
    DetectionSupervisor missing(options("/not-a-detector"));
    check(!missing.start(&error) && !error.empty(), "invalid executable accepted");
}
void nonBlockingAndCleanup() {
    const std::string file = "/tmp/pure_uav_supervisor_test_" + std::to_string(getpid()) + ".pid";
    struct Cleanup {
        const std::string& file;
        ~Cleanup() {
            unlink(file.c_str());
        }
    } cleanup{file};
    pid_t child = -1;
    {
        auto config = options("/bin/sh");
        config.arguments = {"-c", "echo $$ > '" + file + "'; exec sleep 10"};
        DetectionSupervisor process(config);
        std::string error;
        const auto before = Clock::now();
        check(process.start(&error), "persistent detector did not start");
        check(Clock::now() - before < std::chrono::seconds(1), "start blocked application loop");
        const auto deadline = Clock::now() + std::chrono::seconds(2);
        while (child <= 0) {
            std::ifstream in(file);
            in >> child;
            check(Clock::now() < deadline, "child pid was not recorded");
            if (child <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        SuperviseResult result;
        check(!process.finished(result), "running detector stopped early");
        // 模拟相机/VIO 故障的作用域退出：析构必须停子进程并回收监督线程。
    }
    check(kill(child, 0) < 0 && errno == ESRCH, "detector survived application shutdown");
    int status = 0;
    check(waitpid(child, &status, WNOHANG) < 0 && errno == ECHILD, "detector became a zombie");
}
}  // namespace

int main() {
    try {
        normalExitAndRestartLimit();
        nonBlockingAndCleanup();
        std::puts("PASS: detection supervision stays asynchronous, bounds retries and reaps children on application shutdown");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
