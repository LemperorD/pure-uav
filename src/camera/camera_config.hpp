#pragma once

#include <string>

#include "pure_uav/depth_range.hpp"

namespace YAML {
class Node;
}

namespace pure_uav::camera {

inline constexpr char kDefaultCameraConfig[] = "config/camera.yaml";

struct CaptureConfig {
    int width = 0, height = 0, fps = 0;
    bool use_depth = false;
    int ready_timeout_ms = 0;
    std::string vio_config_dir;
    std::string color_calibration_path;
    RangeCameraModel color;
};

std::string locateCameraConfig(const std::string& path);
bool loadCaptureConfig(const YAML::Node& root, CaptureConfig& out, std::string& error);
bool loadCaptureConfig(const std::string& path, CaptureConfig& out, std::string& error);

}  // namespace pure_uav::camera
