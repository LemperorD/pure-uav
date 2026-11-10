// 机载 main：OFFBOARD 之前的飞行流程。
//
// 本版本只做：
//   连接 → 自检 → guidance 配置/相机标定初始化 → 地面参考
//   → arm → AUTO_TAKEOFF → 爬升 → AUTO_LOITER
//   → 零速度预推流 → 切 OFFBOARD
//
// 本版本不启动 guidance 线程、不接检测目标、不启动 VIO/相机/检测，
// 也不实现 OFFBOARD 后的输出仲裁和正式状态机。
// 切进 OFFBOARD 后只保留一个零速度保持循环，保证 setpoint 不断流；
// Ctrl-C 或链路/模式异常时先切 AUTO_LAND，再停止 setpoint。
//
// 配置见 config/main.yaml、config/onboard-guidance.yaml、
// config/guidance-camera.yaml；参数校验失败返回 2，运行失败返回 1。

#include <cstdio>
#include <string>

#include "communicator.hpp"
#include "glue/guidance_session.hpp"
#include "startup/flight_flow.hpp"
#include "startup/flight_safety.hpp"
#include "startup/logging.hpp"
#include "startup/main_config.hpp"

namespace {

using namespace pure_uav;
using namespace pure_uav::app;

struct Args {
    std::string config = "config/main.yaml";
    bool check_config = false;
    bool help = false;
};

void usage(const char* program) {
    std::printf(
        "用法: %s [选项]\n"
        "  --config FILE      主配置，默认 config/main.yaml\n"
        "  --check-config     只校验配置和相机标定，不打开串口/设备\n"
        "  -h, --help         查看帮助\n",
        program);
}

bool parseArgs(int argc, char** argv, Args& args, std::string& error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            args.help = true;
            continue;
        }
        if (arg == "--check-config") {
            args.check_config = true;
            continue;
        }
        if (arg == "--config") {
            if (++i >= argc) {
                error = "--config 缺少参数";
                return false;
            }
            args.config = argv[i];
            continue;
        }
        error = "未知参数: " + arg;
        return false;
    }
    return true;
}

bool checkConfigOnly(const MainConfig& cfg) {
    const std::string driver_path = findConfigFile(cfg.driver_config);
    CommConfig comm_cfg;
    if (!loadCommConfig(driver_path, comm_cfg)) {
        std::fprintf(stderr, "通信配置加载失败: %s\n", driver_path.c_str());
        return false;
    }

    // 声明顺序保证 guidance 先于 unused_comm 析构；attach 只构造引用，不打开串口。
    Communicator unused_comm;
    GuidanceSession guidance;
    std::string error;
    const std::string guidance_path = findConfigFile(cfg.guidance_config);
    if (!guidance.initParams(guidance_path, error)) {
        std::fprintf(stderr, "guidance 参数加载失败: %s\n", error.c_str());
        return false;
    }

    const std::string camera_path = findConfigFile(cfg.camera_calibration);
    const CameraLoadResult camera = guidance.initCamera(camera_path, error);
    if (camera == CameraLoadResult::kError) {
        std::fprintf(stderr, "相机标定加载失败: %s\n", error.c_str());
        return false;
    }
    if (camera == CameraLoadResult::kOk) {
        // GuidanceLoop::init() 不依赖真实串口；这里用未启动的 Communicator
        // 只验证参数和相机模型，不打开设备。
        if (!guidance.attach(unused_comm, error)) {
            std::fprintf(stderr, "GuidanceLoop 初始化失败: %s\n", error.c_str());
            return false;
        }
    }

    std::printf("配置校验通过\n");
    std::printf("  guidance loop: %s\n", guidance.readyForGuidance() ? "initialized" : "not initialized（相机未标定）");
    std::printf("  driver: %s (%s @ %d)\n", driver_path.c_str(), comm_cfg.device.c_str(), comm_cfg.baudrate);
    std::printf("  guidance: %s\n", guidance_path.c_str());
    std::printf("  camera: %s (%s)\n", camera_path.c_str(), guidance.cameraReason().c_str());
    std::printf("  takeoff: %.2f m, timeout=%d s\n", cfg.takeoff.altitude_m, cfg.takeoff.timeout_s);
    std::printf("  offboard: %d Hz, prestream=%.2f s\n", cfg.offboard.rate_hz, cfg.offboard.prestream_s);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    std::string parse_error;
    if (!parseArgs(argc, argv, args, parse_error)) {
        std::fprintf(stderr, "%s\n", parse_error.c_str());
        usage(argv[0]);
        return 2;
    }
    if (args.help) {
        usage(argv[0]);
        return 0;
    }

    const std::string config_path = findConfigFile(args.config);
    MainConfig cfg;
    std::string config_error;
    if (!loadMainConfig(config_path, cfg, config_error)) {
        std::fprintf(stderr, "配置加载失败: %s\n", config_error.c_str());
        return 2;
    }

    if (args.check_config) {
        return checkConfigOnly(cfg) ? 0 : 2;
    }

    Logger log;
    std::string log_error;
    if (!log.open(log_error)) {
        std::fprintf(stderr, "日志文件打开失败，继续使用控制台日志: %s\n", log_error.c_str());
    }
    log.info(Phase::kConfig, "配置加载成功: " + config_path);

    installSignalHandlers();

    Communicator comm;
    GuidanceSession guidance;
    FlightState state;
    OffboardZeroStreamer streamer(comm, cfg.offboard.rate_hz);

    if (!waitLink(comm, cfg, log, state)) {
        comm.stop();
        return interrupted() ? 0 : 1;
    }

    // 早做 guidance 的参数/标定初始化；本阶段不启动 guidance 线程。
    std::string guidance_error;
    const bool params_ok = guidance.initParams(findConfigFile(cfg.guidance_config), guidance_error);
    if (!params_ok) {
        log.error(Phase::kGuidance, "参数初始化失败: " + guidance_error);
    }

    const CameraLoadResult camera_result = guidance.initCamera(findConfigFile(cfg.camera_calibration), guidance_error);
    if (camera_result == CameraLoadResult::kError) {
        log.error(Phase::kGuidance, "相机标定初始化失败: " + guidance_error);
    } else if (camera_result == CameraLoadResult::kNotCalibrated) {
        log.warn(Phase::kGuidance, "相机尚未标定: " + guidance.cameraReason());
    }

    if (params_ok && camera_result == CameraLoadResult::kOk) {
        if (!guidance.attach(comm, guidance_error)) {
            log.error(Phase::kGuidance, "GuidanceLoop 初始化失败: " + guidance_error);
        } else {
            log.info(Phase::kGuidance, "GuidanceLoop 已初始化（未启动线程）: " + guidance.cameraReason());
        }
    }
    log.info(Phase::kGuidance, "状态: params_ok=" + std::to_string(static_cast<int>(guidance.paramsOk())) + " camera_ok=" + std::to_string(static_cast<int>(guidance.cameraOk())) + " ready=" + std::to_string(static_cast<int>(guidance.readyForGuidance())) + " reason=" + guidance.cameraReason());

    if (!runPreflight(comm, cfg, log, state)) {
        safeAbort(comm, cfg, log, state, nullptr);
        comm.stop();
        return interrupted() ? 0 : 1;
    }

    if (!params_ok || guidance.cameraFatal()) {
        log.error(Phase::kGuidance, "guidance 配置/标定存在致命错误，禁止解锁");
        safeAbort(comm, cfg, log, state, nullptr);
        comm.stop();
        return 1;
    }
    if (!guidance.readyForGuidance()) {
        log.warn(Phase::kGuidance, "guidance 未就绪：本阶段只飞 OFFBOARD 零速度，不启动 guidance");
    }

    if (!captureGroundReference(comm, cfg, log, state)) {
        safeAbort(comm, cfg, log, state, nullptr);
        comm.stop();
        return interrupted() ? 0 : 1;
    }

    if (!arm(comm, cfg, log)) {
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }
    if (!takeoff(comm, cfg, log, state)) {
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }
    if (!waitAltitude(comm, cfg, log, state)) {
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }
    if (!hover(comm, cfg, log, state)) {
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }
    if (!enterOffboard(comm, cfg, log, streamer)) {
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }

    const bool hold_ok = holdOffboard(comm, cfg, log, streamer, state);
    if (!hold_ok) {
        log.warn(Phase::kOffboard, "OFFBOARD 保持结束，进入降落");
    }

    if (!landAndDisarm(comm, cfg, log, state, streamer)) {
        log.error(Phase::kLand, "自动降落失败，进入安全退出");
        safeAbort(comm, cfg, log, state, &streamer);
        comm.stop();
        return 1;
    }

    comm.stop();
    log.info(Phase::kDone, "main OFFBOARD 前流程完成");
    return 0;
}
