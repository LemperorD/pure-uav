#include "startup/app_options.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace pure_uav::app {

void printAppUsage(const char* name) {
    std::printf(
        "用法: %s [选项]\n"
        "连接 PX4、连接 camera_server 和运行 VIO，按固定节拍发布位姿；Ctrl-C 停止。\n"
        "飞控心跳就绪后启动检测子进程；--mission 显式启用主任务。\n"
        "  --config FILE         应用调度配置（默认 config/app.yaml）\n"
        "  --vio-config FILE     VIO 运行配置（默认由 app.yaml 指向 config/vio.yaml）\n"
        "  --check-config        校验并打印实际配置后退出，不打开设备\n"
        "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
        "  --config-dir DIR      覆盖 VIO 标定目录（默认由 vio.yaml 指定）\n"
        "  --shm-vio NAME        camera_server 的 VIO 通道（默认 /pure_uav_camera_vio）\n"
        "  --shm-timeout-ms N    等待相机服务的超时（默认 5000 ms）\n"
        "  --no-send             检查照跑，禁止发送位姿、任务指令和飞控动作请求\n"
        "  --mission             启用任务：定位就绪后自检、起飞、搜索、撞板和返航\n"
        "  --mission-config FILE 任务参数（默认 config/mission.yaml）\n"
        "  --guidance-config FILE 制导/彩色相机标定参数；未就绪时只搜索不撞板\n"
        "  --detect-bin FILE     检测子程序路径（默认与 main 同目录的 detect_rt）\n"
        "  --detection-config FILE  传给检测子进程的 --config 路径\n"
        "  --detect-arg ARG      可重复，原样传给检测子进程的运行参数\n"
        "  --no-detection        仅运行通信和 VIO，不拉起检测子进程\n"
        "  -h, --help            查看帮助\n"
        "  --blackbox 为后续黑匣子预留；SIGUSR1 请求任务返航。\n",
        name);
}

AppOptions parseAppOptions(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            options.help = true;
            return options;
        }
        if (arg == "--mission") {
            options.mission = true;
            continue;
        }
        if (arg == "--check-config") {
            options.check_config = true;
            continue;
        }
        if (arg == "--no-detection") {
            options.no_detection = true;
            continue;
        }
        if (arg == "--detect-arg") {
            if (++i == argc) {
                throw std::invalid_argument(arg + " 缺少参数值");
            }
            options.detect_args.emplace_back(argv[i]);
            continue;
        }
        if (arg == "--no-send") {
            options.no_send = true;
            continue;
        }
        if (arg == "--shm-timeout-ms") {
            if (++i == argc) {
                throw std::invalid_argument(arg + " 缺少参数值");
            }
            size_t used = 0;
            options.shm_timeout_ms = std::stoi(argv[i], &used);
            if (used != std::strlen(argv[i]) || options.shm_timeout_ms < 0 || options.shm_timeout_ms > 60000) {
                throw std::invalid_argument("--shm-timeout-ms 只接受 0..60000");
            }
            continue;
        }
        std::string* value = nullptr;
        if (arg == "--config") {
            value = &options.config;
        } else if (arg == "--mission-config") {
            value = &options.mission_config;
        } else if (arg == "--vio-config") {
            value = &options.vio_config_path;
        } else if (arg == "--driver-config") {
            value = &options.driver_config_path;
        } else if (arg == "--shm-vio") {
            value = &options.shm_vio;
        } else if (arg == "--config-dir") {
            value = &options.vio_calibration_dir;
        } else if (arg == "--guidance-config") {
            value = &options.guidance;
        } else if (arg == "--detect-bin") {
            value = &options.detect_bin;
        } else if (arg == "--detection-config") {
            value = &options.detection_config;
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

}  // namespace pure_uav::app
