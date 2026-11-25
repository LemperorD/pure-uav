// 仅接 Orbbec 相机即可测距；不打开飞控串口，不依赖 IMU/VIO 初始化。
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include "orbbec_camera.hpp"
#include "pure_uav/depth_range.hpp"

namespace {
std::atomic<bool> stop_requested{false};
void onSignal(int) {
    stop_requested.store(true);
}
void usage() {
    std::cout << "depth_range_probe --pixel U V [--width 1920 --height 1080 --fps 30]\n"
                 "                  [--duration 10] [--out out/depth_range.csv]\n"
                 "                  [--color-calib path/to/kalibr_imucam_chain.yaml]\n"
                 "只需相机；U/V 为原始彩色图像坐标。CSV 时间戳是相机设备时间，不是 FC boot 时间。\n";
}
}  // namespace
int main(int argc, char** argv) {
    pure_uav::OrbbecCamera::Config cfg;
    cfg.auto_tune_exposure = false;  // 测距不需要进场亮度调优
    double u = 0, v = 0, duration_s = 10;
    bool pixel_given = false;
    std::string output = "out/depth_range.csv";
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto need = [&]() -> std::string {
                if (++i >= argc) throw std::runtime_error(arg + " 缺少参数");
                return argv[i];
            };
            if (arg == "--help" || arg == "-h") {
                usage();
                return 0;
            }
            if (arg == "--pixel") {
                u = std::stod(need());
                v = std::stod(need());
                pixel_given = true;
            } else if (arg == "--width")
                cfg.width = std::stoi(need());
            else if (arg == "--height")
                cfg.height = std::stoi(need());
            else if (arg == "--fps")
                cfg.fps = std::stoi(need());
            else if (arg == "--duration")
                duration_s = std::stod(need());
            else if (arg == "--out")
                output = need();
            else if (arg == "--color-calib") {
                cfg.range_color_calibration_path = need();
                if (cfg.range_color_calibration_path.empty()) throw std::runtime_error("--color-calib 不能为空");
            } else
                throw std::runtime_error("未知参数：" + arg);
        }
        if (!pixel_given || !std::isfinite(u) || !std::isfinite(v) || u < 0 || v < 0 || u > cfg.width - 1 || v > cfg.height - 1 || !std::isfinite(duration_s) || duration_s <= 0 || cfg.fps <= 0 || output.empty()) throw std::runtime_error("需要有效彩色像素、正时长/帧率和输出路径");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        usage();
        return 2;
    }
    if (!cfg.range_color_calibration_path.empty()) {
        pure_uav::RangeCameraModel checked;
        std::string error;
        if (!pure_uav::loadRangeColorCalibration(cfg.range_color_calibration_path, cfg.width, cfg.height, checked, &error)) {
            std::cerr << error << '\n';
            return 2;
        }
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    const auto parent = std::filesystem::path(output).parent_path();
    std::error_code ec;
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);
    std::ofstream csv(output);
    if (!csv) {
        std::cerr << "无法打开 " << output << '\n';
        return 1;
    }
    csv << "color_device_us,depth_device_us,u_px,v_px,has_range,range_m\n";
    csv << std::fixed << std::setprecision(6);
    std::mutex mutex;
    cv::Mat latest_depth;
    int64_t latest_color_us = 0, latest_depth_us = 0;
    uint64_t sequence = 0;
    pure_uav::OrbbecCamera camera;
    std::string error;
    if (!camera.start(
            cfg,
            [&](const cv::Mat&, const cv::Mat& depth, int64_t, int64_t color_us, int64_t depth_us) {
                std::lock_guard<std::mutex> lock(mutex);
                latest_depth = depth;  // SDK 包装层已复制像素；引用计数持有这一帧
                latest_color_us = color_us;
                latest_depth_us = depth_us;
                ++sequence;
            },
            &error)) {
        std::cerr << "相机启动失败：" << error << '\n';
        return 1;
    }
    pure_uav::DepthRangeProjector projector;
    if (!projector.configure(camera.rangeCalibration())) {
        std::cerr << "完整测距标定不可用；检查 SDK/设备标定与两路分辨率。\n";
        camera.stop();
        return 1;
    }
    const auto start = std::chrono::steady_clock::now();
    auto last_report = start - std::chrono::seconds(1);
    uint64_t consumed = 0, frames = 0, valid = 0;
    while (!stop_requested.load() && camera.running() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration_s) {
        cv::Mat depth;
        int64_t color_us = 0, depth_us = 0;
        bool fresh = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (consumed != sequence) {
                consumed = sequence;
                depth = latest_depth;
                color_us = latest_color_us;
                depth_us = latest_depth_us;
                fresh = true;
            }
        }
        if (!fresh) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        projector.update(depth, depth_us > 0 ? static_cast<uint64_t>(depth_us) : 0);
        const auto distance = projector.sample(u, v, color_us > 0 ? static_cast<uint64_t>(color_us) : 0);
        ++frames;
        if (distance.has_range) ++valid;
        csv << color_us << ',' << depth_us << ',' << u << ',' << v << ',' << (distance.has_range ? 1 : 0) << ',' << distance.range_m << '\n';
        csv.flush();
        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            std::cout << "has_range=" << (distance.has_range ? "true" : "false") << " range_m=" << distance.range_m << '\n';
            last_report = now;
        }
    }
    const bool camera_ok = camera.running();
    camera.stop();
    std::cout << "测距帧 " << frames << "，有效 " << valid << "，输出 " << output << '\n';
    return camera_ok && frames > 0 && valid > 0 && csv.good() ? 0 : 1;
}
