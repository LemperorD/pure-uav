// 唯一相机采集进程：Orbbec 设备只在这里被初始化一次。
// main 内的 VIO（或独立 vio_live）与 detect_rt 从两条共享内存通道取帧。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>

#include "camera_shm.hpp"
#include "frame_publisher.hpp"
#include "orbbec_camera.hpp"

namespace {
using pure_uav::OrbbecCamera;
using pure_uav::camera::FramePublisher;
using pure_uav::camera::PublisherConfig;
using pure_uav::camera::StreamMeta;

volatile std::sig_atomic_t interrupted = 0;
void onSignal(int /*unused*/) {
    interrupted = 1;
}

struct ServerConfig {
    OrbbecCamera::Config camera;
    PublisherConfig publisher;
    double status_period_s = 1.0;
    int ready_timeout_ms = 5000;
};

void usage(const char* name) {
    std::fprintf(stderr,
                 "用法: %s [选项]\n"
                 "  相机服务：独占 Orbbec 采集，向 VIO / 检测分发共享内存帧。\n"
                 "  --config FILE       配置文件（默认 config/camera.yaml）\n"
                 "  --check-config      只校验配置，不打开相机/不创建通道\n"
                 "  --vio-channel NAME  VIO 通道名（覆盖配置）\n"
                 "  --color-channel NAME 彩色通道名（覆盖配置）\n"
                 "  --color-max-hz N    彩色通道限频（覆盖配置）\n"
                 "  -h, --help          帮助\n",
                 name);
}

template <typename T>
void read(const YAML::Node& node, const char* key, T& out) {
    if (node && node[key]) {
        out = node[key].as<T>();
    }
}

std::string locateConfig(const std::string& path) {
    if (std::filesystem::exists(path)) {
        return path;
    }
    std::error_code ec;
    auto base = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) {
        return path;
    }
    for (int level = 0; level < 6 && base.has_parent_path(); ++level) {
        base = base.parent_path();
        const auto candidate = base / path;
        if (std::filesystem::exists(candidate)) {
            return candidate.string();
        }
    }
    return path;
}

bool loadConfig(const YAML::Node& root, ServerConfig& out, std::string& error) {
    try {
        const YAML::Node cam = root["camera"];
        if (!cam || !cam.IsMap()) {
            error = "config/camera.yaml: missing 'camera' section";
            return false;
        }
        read(cam, "width", out.camera.width);
        read(cam, "height", out.camera.height);
        read(cam, "fps", out.camera.fps);
        read(cam, "use_depth", out.camera.use_depth);
        read(cam, "range_color_calibration_path", out.camera.range_color_calibration_path);
        read(cam, "aggregate_mode", out.camera.aggregate_mode);
        read(cam, "auto_exposure", out.camera.auto_exposure);
        read(cam, "exposure", out.camera.exposure);
        read(cam, "gain", out.camera.gain);
        read(cam, "white_balance", out.camera.white_balance);
        read(cam, "warmup_frames", out.camera.warmup_frames);
        read(cam, "auto_tune_exposure", out.camera.auto_tune_exposure);
        read(cam, "tune_expected_rate_dps", out.camera.tune_expected_rate_dps);
        read(cam, "tune_blur_budget_px", out.camera.tune_blur_budget_px);
        read(cam, "tune_max_exposure_ms", out.camera.tune_max_exposure_ms);
        read(cam, "tune_max_gain", out.camera.tune_max_gain);
        read(cam, "tune_allow_dark", out.camera.tune_allow_dark);

        if (!out.camera.range_color_calibration_path.empty()) {
            out.camera.range_color_calibration_path = locateConfig(out.camera.range_color_calibration_path);
            pure_uav::RangeCameraModel checked;
            if (!pure_uav::loadRangeColorCalibration(out.camera.range_color_calibration_path, out.camera.width, out.camera.height, checked, &error)) return false;
        }

        const YAML::Node channels = root["channels"];
        read(channels, "vio_name", out.publisher.vio_name);
        read(channels, "color_name", out.publisher.color_name);
        read(channels, "slots", out.publisher.slots);
        int vio_bytes = 0;
        int color_bytes = 0;
        read(channels, "vio_slot_bytes", vio_bytes);
        read(channels, "color_slot_bytes", color_bytes);
        if (vio_bytes > 0) {
            out.publisher.vio_payload_bytes = static_cast<size_t>(vio_bytes);
        }
        if (color_bytes > 0) {
            out.publisher.color_payload_bytes = static_cast<size_t>(color_bytes);
        }
        read(channels, "color_max_hz", out.publisher.color_max_hz);
        read(channels, "ready_timeout_ms", out.ready_timeout_ms);

        const YAML::Node runtime = root["runtime"];
        read(runtime, "status_period_s", out.status_period_s);

        if (out.camera.width < 320 || out.camera.width > 4096 || out.camera.height < 240 || out.camera.height > 2160 || out.camera.fps < 1 || out.camera.fps > 120) {
            error = "config/camera.yaml: camera resolution/fps out of range";
            return false;
        }
        if (out.publisher.vio_name.empty() || out.publisher.vio_name.front() != '/' || out.publisher.color_name.empty() || out.publisher.color_name.front() != '/' || out.publisher.vio_name == out.publisher.color_name) {
            error = "config/camera.yaml: channel names must be distinct and start with '/'";
            return false;
        }
        if (out.publisher.slots < 2 || out.publisher.color_max_hz < 1 || out.publisher.color_max_hz > 60 || out.publisher.vio_payload_bytes < 1024U * 1024U || out.publisher.color_payload_bytes < 64U * 1024U) {
            error = "config/camera.yaml: invalid slots / color_max_hz / slot bytes";
            return false;
        }
        if (out.status_period_s < 0.1 || out.status_period_s > 60.0) {
            error = "config/camera.yaml: status_period_s out of range";
            return false;
        }
        error.clear();
        return true;
    } catch (const std::exception& e) {
        error = std::string("config/camera.yaml: ") + e.what();
        return false;
    }
}

StreamMeta buildStreamMeta(const ServerConfig& cfg, const OrbbecCamera& camera) {
    StreamMeta meta;
    meta.width = static_cast<uint32_t>(cfg.camera.width);
    meta.height = static_cast<uint32_t>(cfg.camera.height);
    meta.fps = static_cast<uint32_t>(cfg.camera.fps);
    meta.has_depth = cfg.camera.use_depth ? 1U : 0U;
    // 实际编码格式逐帧写在 FrameMeta.color_encoding；这里只给期望值。
    meta.color_encoding = static_cast<uint32_t>(pure_uav::ColorEncoding::jpeg);

    const auto range = camera.rangeCalibration();
    meta.range_calibration_valid = range.valid() ? 1U : 0U;
    meta.color_fx = range.color.fx;
    meta.color_fy = range.color.fy;
    meta.color_cx = range.color.cx;
    meta.color_cy = range.color.cy;
    std::copy(range.depth.distortion.begin(), range.depth.distortion.end(), meta.depth_distortion);
    std::copy(range.color.distortion.begin(), range.color.distortion.end(), meta.color_distortion);
    const auto depth = camera.depthInfo();
    meta.fx = depth.fx;
    meta.fy = depth.fy;
    meta.cx = depth.cx;
    meta.cy = depth.cy;
    meta.depth_width = static_cast<uint32_t>(depth.width);
    meta.depth_height = static_cast<uint32_t>(depth.height);
    for (int index = 0; index < 9; ++index) {
        meta.R_c2d[index] = depth.R_c2d[index];
    }
    for (int index = 0; index < 3; ++index) {
        meta.t_mm[index] = depth.t_mm[index];
    }
    return meta;
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/camera.yaml";
    bool check_config = false;
    std::string vio_override;
    std::string color_override;
    int hz_override = 0;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        }
        if (arg == "--check-config") {
            check_config = true;
            continue;
        }
        if (index + 1 >= argc) {
            std::fprintf(stderr, "[camera_server] 参数缺少值: %s\n", arg.c_str());
            return 2;
        }
        const std::string value = argv[++index];
        if (arg == "--config") {
            config_path = value;
        } else if (arg == "--vio-channel") {
            vio_override = value;
        } else if (arg == "--color-channel") {
            color_override = value;
        } else if (arg == "--color-max-hz") {
            hz_override = std::atoi(value.c_str());
        } else {
            std::fprintf(stderr, "[camera_server] 未知参数: %s\n", arg.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    ServerConfig cfg;
    std::string error;
    YAML::Node root;
    try {
        root = YAML::LoadFile(locateConfig(config_path));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[camera_server] 读取配置失败: %s\n", e.what());
        return 2;
    }
    if (!loadConfig(root, cfg, error)) {
        std::fprintf(stderr, "[camera_server] %s\n", error.c_str());
        return 2;
    }
    if (!vio_override.empty()) {
        cfg.publisher.vio_name = vio_override;
    }
    if (!color_override.empty()) {
        cfg.publisher.color_name = color_override;
    }
    if (hz_override > 0) {
        cfg.publisher.color_max_hz = hz_override;
    }
    if (check_config) {
        std::printf("[camera_server] 配置有效: %dx%d@%d depth=%d vio=%s color=%s color_hz=%d\n", cfg.camera.width, cfg.camera.height, cfg.camera.fps, cfg.camera.use_depth ? 1 : 0, cfg.publisher.vio_name.c_str(), cfg.publisher.color_name.c_str(), cfg.publisher.color_max_hz);
        return 0;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    FramePublisher publisher;
    if (!publisher.create(cfg.publisher, &error)) {
        std::fprintf(stderr, "[camera_server] 创建共享内存通道失败: %s\n", error.c_str());
        return 1;
    }
    std::fprintf(stderr, "[camera_server] 通道已创建: %s / %s（等待相机就绪）\n", cfg.publisher.vio_name.c_str(), cfg.publisher.color_name.c_str());

    OrbbecCamera camera;
    if (!camera.startWithFrames(cfg.camera, [&publisher](const OrbbecCamera::CameraFrames& frames) { publisher.publish(frames); }, &error)) {
        std::fprintf(stderr, "[camera_server] 相机启动失败（设备可能被其他进程占用）: %s\n", error.c_str());
        publisher.stop();
        return 1;
    }

    // SDK 的深度内参/外参只有启动后才能读出，补齐静态元数据再让消费者进入。
    const StreamMeta meta = buildStreamMeta(cfg, camera);
    publisher.activate(meta);
    std::fprintf(stderr, "[camera_server] 已就绪: %dx%d@%d 深度内参 %s (fx=%.1f)，两通道可连接\n", cfg.camera.width, cfg.camera.height, cfg.camera.fps, meta.fx > 0 ? "有效" : "不可用", meta.fx);

    const auto period = std::chrono::milliseconds(static_cast<int>(cfg.status_period_s * 1000.0));
    auto last_status = std::chrono::steady_clock::now() - period;
    int exit_code = 0;
    while (interrupted == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!camera.running()) {
            std::fprintf(stderr, "[camera_server] 采集线程已退出，停止服务（由外部托管方负责重启）\n");
            exit_code = 1;
            break;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_status >= period) {
            last_status = now;
            const auto stats = publisher.stats();
            std::fprintf(stderr, "[camera_server] vio=%llu color=%llu limited=%llu dropped=%llu shm_dropped=%llu\n", static_cast<unsigned long long>(stats.vio_frames), static_cast<unsigned long long>(stats.color_frames), static_cast<unsigned long long>(stats.color_rate_limited),
                         static_cast<unsigned long long>(stats.dropped), static_cast<unsigned long long>(publisher.channelDropped()));
        }
    }

    publisher.markClosing();
    camera.stop();  // join 采集线程，确保没有回调还在写共享内存
    publisher.stop();
    std::fprintf(stderr, "[camera_server] 已停止，共享内存通道已清理\n");
    return exit_code;
}
