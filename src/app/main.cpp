// 机载主程序：等飞控心跳后拉起并监督检测子进程。
// 相机只由 camera_server 初始化一次；相机服务与 VIO 由外部托管启动。
// 本程序不解析检测配置、不连接共享内存、不参与取帧与推理。
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
#include <utility>
#include <vector>

#include "communicator.hpp"
#include "detection_process.hpp"
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
    std::string detect_bin;
    std::string detection_config;
    std::vector<std::string> detect_args;
    bool no_send = false;
    bool no_detection = false;
    bool help = false;
};

void usage(const char* name) {
    std::printf(
        "用法: %s [选项]\n"
        "等飞控心跳后拉起并监督检测子进程；Ctrl-C 停止。\n"
        "相机服务与 VIO 由外部托管启动，本程序不拉起它们。\n"
        "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
        "  --config-dir DIR      VIO 配置目录（默认 gemini_1920x1080）\n"
        "  --guidance-config FILE 制导参数覆盖（相机默认读取 VIO 标定）\n"
        "  --blackbox FILE       退出时保存制导黑匣子\n"
        "  --no-send             运行计算但不发送位姿和制导指令\n"
        "  --detect-bin FILE     检测子程序路径（默认与 main 同目录的 detect_rt）\n"
        "  --detection-config FILE  传给检测子进程的 --config 路径\n"
        "  --detect-arg ARG      重复传入，原样转发给检测子进程（仅运行参数）\n"
        "  --no-detection        不拉起检测子进程，只保留通信流程\n"
        "  -h, --help            查看帮助\n"
        "\n检测算法参数（模型/阈值/类别/限频/帧龄）唯一真源为 config/detector-rt.yaml，\n"
        "由检测子进程自行读取，本程序不解析、不覆盖。\n",
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
        if (arg == "--no-detection") {
            options.no_detection = true;
            continue;
        }
        if (arg == "-h" || arg == "--help") {
            options.help = true;
            continue;
        }
        if (arg == "--detect-arg") {
            if (++i == argc) {
                throw std::invalid_argument(arg + " 缺少参数值");
            }
            options.detect_args.emplace_back(argv[i]);
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
        } else if (arg == "--detect-bin") {
            value = &options.detect_bin;
        } else if (arg == "--detection-config") {
            value = &options.detection_config;
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

// main 与 detect_rt 都由 CMake 输出到同一个 bin/ 目录，优先取 argv[0] 同目录。
std::string locateDetectBinary(const std::string& configured, const char* argv0) {
    if (!configured.empty()) {
        return configured;
    }
    const auto executable = [](const std::filesystem::path& path) {
        std::error_code ec;
        return std::filesystem::is_regular_file(path, ec) && access(path.c_str(), X_OK) == 0;
    };
    if (argv0 != nullptr) {
        const auto sibling = std::filesystem::path(argv0).parent_path() / "detect_rt";
        if (executable(sibling)) {
            return sibling.string();
        }
    }
    const std::filesystem::path relative("bin/detect_rt");
    if (executable(relative)) {
        return relative.string();
    }

    char buffer[4096];
    const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) {
        return {};
    }
    buffer[length] = '\0';
    std::filesystem::path dir = std::filesystem::path(buffer).parent_path();
    for (int level = 0; level < 6 && !dir.empty(); ++level) {
        const auto candidate = dir / "bin/detect_rt";
        if (executable(candidate)) {
            return candidate.string();
        }
        const auto parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }
    return {};
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
    Options options;
    try {
        options = parseOptions(argc, argv);
    } catch (const std::exception& failure) {
        std::fprintf(stderr, "[main] %s\n", failure.what());
        usage(argv[0]);
        return 2;
    }
    if (options.help) {
        usage(argv[0]);
        return 0;
    }

    // 当前文件定义了 onSignal 却从未注册，补上以保证 Ctrl-C 能停止子进程。
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    Communicator comm;

    std::printf("等待飞控心跳...\n");
    while (!comm.isConnected() && interrupted == 0) {
        usleep(100000);
    }
    if (interrupted != 0) {
        std::fprintf(stderr, "[main] 收到停止信号，退出\n");
        return 0;
    }
    std::printf("已连接，固件：%s，模式：%s\n", comm.getAutopilotNameCn(), comm.getFlightModeNameCn());

    // 读一次姿态示例
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("姿态 roll=%.3f pitch=%.3f yaw=%.3f\n", a.roll, a.pitch, a.yaw);
    }

#ifdef PURE_APP_HAS_DETECTION
    if (options.no_detection) {
        std::fprintf(stderr, "[main] --no-detection：不拉起检测子进程\n");
        return 0;
    }
    const std::string detect_bin = locateDetectBinary(options.detect_bin, argv[0]);
    if (detect_bin.empty()) {
        std::fprintf(stderr, "[main] 找不到 bin/detect_rt，可用 --detect-bin FILE 指定路径\n");
        return 1;
    }

    // 只透传运行参数与可选配置路径；检测算法参数由子进程从
    // config/detector-rt.yaml 自行读取，本程序不解析、不覆盖。
    pure_uav::app::DetectionProcessOptions process_options;
    process_options.executable = detect_bin;
    if (!options.detection_config.empty()) {
        process_options.arguments.emplace_back("--config");
        process_options.arguments.push_back(options.detection_config);
    }
    for (const auto& argument : options.detect_args) {
        process_options.arguments.push_back(argument);
    }

    pure_uav::app::DetectionProcess process(std::move(process_options));
    std::string error;
    if (!process.start(&error)) {
        std::fprintf(stderr, "[main] %s\n", error.c_str());
        return 1;
    }
    const auto result = process.supervise([] { return interrupted != 0; });
    switch (result.status) {
        case pure_uav::app::SuperviseStatus::exited:
        case pure_uav::app::SuperviseStatus::stopped:
            return 0;
        case pure_uav::app::SuperviseStatus::restart_limit:
            std::fprintf(stderr, "[main] 检测子进程重启 %llu 次后仍异常退出，交由外部托管方处置\n", static_cast<unsigned long long>(result.restarts));
            return 1;
        case pure_uav::app::SuperviseStatus::error:
            std::fprintf(stderr, "[main] 监督检测子进程失败: %s\n", result.error.c_str());
            return 1;
    }
    return 0;
#else
    std::fprintf(stderr, "[main] 本次构建未包含 detection_rt，未拉起检测子进程。\n");
    return 0;
#endif
}
