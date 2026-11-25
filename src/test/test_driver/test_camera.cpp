#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

int main(int argc, char** argv) try {
    // 本工具绕过所有封装直接打开 Orbbec 设备，与 camera_server 互斥。
    // 必须显式确认相机服务已停，避免误开设备导致实时链路启动失败。
    const char* allow = std::getenv("PURE_UAV_ALLOW_DIRECT_CAMERA");
    if (allow == nullptr || std::string(allow) != "1") {
        std::cerr << "[test_camera] 本工具会独占 Orbbec 设备，与 camera_server 冲突。\n"
                     "仅在停用相机服务后手工执行：PURE_UAV_ALLOW_DIRECT_CAMERA=1 bin/test/test_camera\n";
        return 2;
    }

    int color_fps = 60;
    int depth_fps = 60;

    if (argc >= 2) {
        color_fps = std::atoi(argv[1]);
    }

    if (argc >= 3) {
        depth_fps = std::atoi(argv[2]);
    }

    ob::Pipeline pipe;
    auto config = std::make_shared<ob::Config>();

    // Color: 640x480 MJPEG
    auto color_profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);

    auto color_profile = color_profiles->getVideoStreamProfile(640, 480, OB_FORMAT_MJPG, color_fps);

    // Depth: 640x400 Y11
    auto depth_profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);

    auto depth_profile = depth_profiles->getVideoStreamProfile(640, 400, OB_FORMAT_Y11, depth_fps);

    config->enableStream(color_profile);
    config->enableStream(depth_profile);

    std::cout << "Configured:" << std::endl;
    std::cout << "  Color: " << color_profile->width() << "x" << color_profile->height() << " @ " << color_profile->fps() << " FPS" << std::endl;

    std::cout << "  Depth: " << depth_profile->width() << "x" << depth_profile->height() << " @ " << depth_profile->fps() << " FPS" << std::endl;

    pipe.start(config);

    uint64_t color_count = 0;
    uint64_t depth_count = 0;

    auto last = std::chrono::steady_clock::now();

    while (true) {
        auto frameset = pipe.waitForFrames(1000);

        if (!frameset) {
            continue;
        }

        if (frameset->colorFrame()) {
            color_count++;
        }

        if (frameset->depthFrame()) {
            depth_count++;
        }

        auto now = std::chrono::steady_clock::now();

        double dt = std::chrono::duration<double>(now - last).count();

        if (dt >= 1.0) {
            double color_actual = static_cast<double>(color_count) / dt;

            double depth_actual = static_cast<double>(depth_count) / dt;

            std::cout << "Actual FPS | Color: " << color_actual << " | Depth: " << depth_actual << std::endl;

            color_count = 0;
            depth_count = 0;
            last = now;
        }
    }
} catch (const ob::Error& e) {
    std::cerr << "Orbbec error:\n"
              << "function: " << e.getName() << "\n"
              << "args: " << e.getArgs() << "\n"
              << "message: " << e.getMessage() << std::endl;

    return 1;
}
