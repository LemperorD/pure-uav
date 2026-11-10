// 通信示例（不再是运行入口）：相机只由 camera_server 初始化一次。
// 相机服务、VIO 与检测进程均由外部托管启动，本程序不负责拉起它们。
// 生命周期和配置见 assets/docs/app/usage.md。
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "communicator.hpp"
#include "guidance_loop.hpp"
#include "runtime/bounded_queue.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

#ifdef PURE_APP_HAS_VIO
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "fcu_clock.hpp"
#include "live_timing.hpp"
#include "orbbec_camera.hpp"
#include "pure_uav/vio_estimator.hpp"
#endif
#ifdef PURE_APP_HAS_DETECTION
#include "detector.hpp"
#include "target_bridge.hpp"
#endif

namespace {
using namespace pure_uav;
namespace flight = guidance::flight;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int /*unused*/) {
    interrupted = 1;
}
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

struct Options {
    std::string driver = "config/driver.yaml";
    std::string vio = "src/vio/estimator/config/gemini_1920x1080";
    std::string guidance = "config/onboard-guidance.yaml";
    std::string blackbox;
    bool no_send = false;
};

void usage(const char* name) {
    std::printf(
        "用法: %s [选项]\n"
        "同时运行通信、共享相机、检测、VIO、制导线程；Ctrl-C 停止。\n"
        "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
        "  --config-dir DIR      VIO 配置目录（默认 gemini_1920x1080）\n"
        "  --guidance-config FILE 制导参数覆盖（相机默认读取 VIO 标定）\n"
        "  --blackbox FILE       退出时保存制导黑匣子\n"
        "  --no-send             运行计算但不发送位姿和制导指令\n"
        "  -h, --help            查看帮助\n",
        name);
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--no-send") {
            options.no_send = true;
            continue;
        }
        std::string* value = nullptr;
        if (arg == "--driver-config") {
            value = &options.driver;
        } else if (arg == "--config-dir") {
            value = &options.vio;
        } else if (arg == "--guidance-config") {
            value = &options.guidance;
        } else if (arg == "--blackbox") {
            value = &options.blackbox;
        } else {
            throw std::invalid_argument("未知参数：" + arg);
        }
        if (++i == argc) {
            throw std::invalid_argument(arg + " 缺少参数值");
        }
        *value = argv[i];
    }
    return options;
}
}  // namespace

#ifdef PURE_APP_HAS_VIO
struct Frame {
    cv::Mat gray, depth;
    int64_t host_ns = 0;
    uint64_t session = 0;
};

// 线程异常统一回到监督线程；析构时先通知退出，再 join，禁止后台悬空访问资源。
class Workers {
  public:
    std::atomic<bool> stop{false};
    std::atomic<bool> failed{false};
    template <typename F>
    void launch(const char* name, F work) {
        threads_.emplace_back([this, name, work] {
            try {
                work();
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[%s] %s\n", name, error.what());
                failed = true;
                stop = true;
            } catch (...) {
                std::fprintf(stderr, "[%s] 未知异常\n", name);
                failed = true;
                stop = true;
            }
        });
    }
    void join() {
        stop = true;
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }
    std::vector<std::thread> threads_;
};  // class Workers

#endif  // PURE_APP_HAS_VIO

int main(int argc, char** argv) {
    Options options = parseOptions(argc, argv);
    (void)options;  // 预留：后续可按配置加载
    Communicator comm;

    std::printf("等待飞控心跳...\n");
    while (!comm.isConnected()) {
        usleep(100000);
    }
    std::printf("已连接，固件：%s，模式：%s\n", comm.getAutopilotNameCn(), comm.getFlightModeNameCn());

    // 读一次姿态示例
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("姿态 roll=%.3f pitch=%.3f yaw=%.3f\n", a.roll, a.pitch, a.yaw);
    }

    // 旧实现在这里直接 Detector::start() 自采相机，与 vio_live / detect_rt
    // 构成第三个独占入口。现在相机只由 camera_server 打开，本示例不再启动检测。
    std::fprintf(stderr,
                 "[main] 相机只由 camera_server 初始化一次；本示例不再打开设备。\n"
                 "       相机服务、VIO 与检测由外部托管启动，本程序不负责拉起。\n");
    return 0;
}
