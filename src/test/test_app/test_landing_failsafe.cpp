#include <atomic>
#include <cstdio>

#include "communicator.hpp"
#include "runtime/landing_failsafe.hpp"

using namespace pure_uav;
using namespace pure_uav::app;

int main() {
    // 中断优先于任何串口访问，保证 Ctrl-C 不会卡在 20 s 降落等待里。
    std::atomic<bool> interrupted{true};
    Communicator comm;
    if (runLandingFailsafe(comm, 0.1, 0.01, interrupted)) {
        std::fprintf(stderr, "FAIL: interrupted landing should not report success\n");
        return 1;
    }
    std::printf("PASS: landing failsafe interruption\n");
    return 0;
}
