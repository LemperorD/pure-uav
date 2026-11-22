#include "camera_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <utility>

namespace pure_uav::camera {
namespace fs = std::filesystem;

std::string locateCameraConfig(const std::string& path) {
    const fs::path requested(path);
    if (requested.empty() || requested.is_absolute() || fs::exists(requested)) return path;
    std::error_code error;
    auto executable = fs::read_symlink("/proc/self/exe", error);
    if (!error) {
        for (auto dir = executable.parent_path(); !dir.empty(); dir = dir.parent_path()) {
            const auto candidate = dir / requested;
            if (fs::exists(candidate)) return candidate.string();
            if (dir == dir.root_path()) break;
        }
    }
    return path;
}

bool loadCaptureConfig(const YAML::Node& root, CaptureConfig& out, std::string& error) {
    const auto fail = [&](const std::string& why) {
        error = "camera.yaml: " + why;
        return false;
    };
    try {
        const auto camera = root["camera"];
        if (!camera.IsMap()) return fail("missing 'camera' section");
        CaptureConfig config;
        config.width = camera["width"].as<int>();
        config.height = camera["height"].as<int>();
        config.fps = camera["fps"].as<int>();
        config.use_depth = camera["use_depth"].as<bool>();
        if (config.width < 320 || config.width > 4096 || config.height < 240 || config.height > 2160 || config.fps < 1 || config.fps > 120) {
            return fail("camera resolution/fps out of range");
        }
        const auto channels = root["channels"];
        if (!channels || !channels["ready_timeout_ms"]) return fail("missing channels.ready_timeout_ms");
        config.ready_timeout_ms = channels["ready_timeout_ms"].as<int>();
        if (config.ready_timeout_ms < 0 || config.ready_timeout_ms > 60000) return fail("channels.ready_timeout_ms must be in [0, 60000]");

        const auto profile = camera["vio_config_dir"].as<std::string>();
        if (profile.empty()) return fail("camera.vio_config_dir must not be empty");
        const auto dir = fs::weakly_canonical(locateCameraConfig(profile));
        const auto estimator_path = (dir / "estimator_config.yaml").string();
        cv::FileStorage estimator(estimator_path, cv::FileStorage::READ);
        if (!estimator.isOpened()) return fail("cannot read " + estimator_path);
        const auto chain = static_cast<std::string>(estimator["relative_config_imucam"]);
        if (chain.empty()) return fail("missing relative_config_imucam in " + estimator_path);
        config.vio_config_dir = dir.string();
        config.color_calibration_path = (dir / chain).string();
        std::string why;
        if (!loadRangeColorCalibration(config.color_calibration_path, config.width, config.height, config.color, &why)) return fail(why);

        out = std::move(config);
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        return fail(exception.what());
    }
}

bool loadCaptureConfig(const std::string& path, CaptureConfig& out, std::string& error) {
    try {
        return loadCaptureConfig(YAML::LoadFile(locateCameraConfig(path)), out, error);
    } catch (const YAML::Exception& exception) {
        error = "读取相机配置失败: " + path + ": " + exception.what();
        return false;
    }
}

}  // namespace pure_uav::camera
