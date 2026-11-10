#include "detector_config.hpp"

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace pure_uav::detection {
namespace {
namespace fs = std::filesystem;

constexpr const char* kTopKey = "detection-rt";

#ifdef PURE_DETECTION_BACKEND_CPU
constexpr const char* kDefaultModelRel = "assets/models/yolo26n.onnx";
#else
constexpr const char* kDefaultModelRel = "assets/models/yolo26n.nb";
#endif

constexpr std::array<const char*, 9> kRequiredKeys = {"model_path", "imgsz", "num_classes", "conf_thr", "iou_thr", "max_det", "target_class", "max_hz", "max_age_ms"};

bool isKnownKey(const std::string& key) {
    for (const char* candidate : kRequiredKeys) {
        if (key == candidate) {
            return true;
        }
    }
    return false;
}

bool isRemovedLegacyKey(const std::string& key) {
    return key == "model" || key == "confidence_threshold" || key == "iou_threshold" || key == "input_size" || key == "device";
}

// 相对路径先按 cwd 解析，再从可执行文件位置逐级上溯仓库根。
std::string locateExisting(const std::string& relative) {
    std::error_code ec;
    if (fs::exists(relative, ec)) {
        return relative;
    }
    char buffer[4096];
    const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) {
        return relative;
    }
    buffer[length] = '\0';
    fs::path dir = fs::path(buffer).parent_path();
    for (int level = 0; level < 6 && !dir.empty(); ++level) {
        const auto candidate = dir / relative;
        if (fs::exists(candidate, ec)) {
            return candidate.string();
        }
        const auto parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }
    return relative;
}

void requireAllKeys(const YAML::Node& node) {
    for (const char* key : kRequiredKeys) {
        if (!node[key]) {
            throw std::runtime_error(std::string("缺少必填字段 ") + kTopKey + "." + key);
        }
    }
}

bool validateKeys(const YAML::Node& node, std::string& error) {
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        if (isKnownKey(key)) {
            continue;
        }
        error = std::string("未知字段 ") + kTopKey + "." + key;
        if (isRemovedLegacyKey(key)) {
            error += "（旧占位字段已移除，请改用 RuntimeConfig 字段名，见 config/detector-rt.yaml）";
        }
        return false;
    }
    return true;
}

template <class T>
T readValue(const YAML::Node& node, const char* key) {
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception& failure) {
        throw std::runtime_error(std::string(kTopKey) + "." + key + " 取值无效: " + failure.what());
    }
}

template <class T>
void requireRange(T value, T low, T high, const char* key) {
    if (value < low || value > high) {
        throw std::runtime_error(std::string(kTopKey) + "." + key + " 超出范围 [" + std::to_string(low) + ", " + std::to_string(high) + "]");
    }
}

void requireThreshold(float value, const char* key) {
    if (!std::isfinite(value) || value <= 0.0F || value > 1.0F) {
        throw std::runtime_error(std::string(kTopKey) + "." + key + " 必须在 (0,1] 内");
    }
}

}  // namespace

std::string locateDetectorConfig(const std::string& path) {
    return locateExisting(path);
}

std::string resolveModelPath(const std::string& configured) {
    const std::string relative = (configured.empty() || configured == "auto") ? std::string(kDefaultModelRel) : configured;
    if (fs::path(relative).is_absolute()) {
        return relative;
    }
    return locateExisting(relative);
}

bool loadDetectorConfig(const std::string& path, RuntimeConfig& out, std::string& error) {
    try {
        const std::string located = locateDetectorConfig(path);
        std::error_code ec;
        if (!fs::is_regular_file(located, ec)) {
            error = "找不到检测配置文件: " + located + "（可用 --config 指定路径）";
            return false;
        }

        const YAML::Node root = YAML::LoadFile(located);
        if (!root || !root.IsMap()) {
            error = "config/detector-rt.yaml 顶层必须是映射";
            return false;
        }
        const YAML::Node node = root[kTopKey];
        if (!node || !node.IsMap()) {
            error = std::string("config/detector-rt.yaml 缺少 ") + kTopKey + ": 段";
            return false;
        }
        if (!validateKeys(node, error)) {
            return false;
        }
        requireAllKeys(node);

        RuntimeConfig config;
        const auto configured_model = readValue<std::string>(node, "model_path");
        config.imgsz = readValue<int>(node, "imgsz");
        config.num_classes = readValue<int>(node, "num_classes");
        config.conf_thr = readValue<float>(node, "conf_thr");
        config.iou_thr = readValue<float>(node, "iou_thr");
        config.max_det = readValue<int>(node, "max_det");
        config.target_class = readValue<int>(node, "target_class");
        config.max_hz = readValue<int>(node, "max_hz");
        config.max_age_ms = readValue<int>(node, "max_age_ms");

        requireRange(config.imgsz, 640, 640, "imgsz");
        requireRange(config.num_classes, 1, 1000, "num_classes");
        requireThreshold(config.conf_thr, "conf_thr");
        requireThreshold(config.iou_thr, "iou_thr");
        requireRange(config.max_det, 1, kMaxDetections, "max_det");
        requireRange(config.max_hz, 1, 60, "max_hz");
        requireRange(config.max_age_ms, 1, 2000, "max_age_ms");
        requireRange(config.target_class, -1, config.num_classes - 1, "target_class");

        config.model_path = resolveModelPath(configured_model);
        std::string validation_error;
        if (!validateRuntimeConfig(config, validation_error)) {
            error = std::string(kTopKey) + ": " + validation_error;
            return false;
        }
        if (!fs::is_regular_file(config.model_path, ec)) {
            error = "检测模型文件不存在: " + config.model_path + "（可改 model_path 为显式路径）";
            return false;
        }

        out = config;
        error.clear();
        return true;
    } catch (const std::exception& failure) {
        error = std::string("config/detector-rt.yaml: ") + failure.what();
        return false;
    }
}

std::string describeDetectorConfig(const RuntimeConfig& config) {
    std::ostringstream text;
    text << "model=" << config.model_path << " imgsz=" << config.imgsz << " classes=" << config.num_classes << " conf=" << config.conf_thr << " iou=" << config.iou_thr << " max_det=" << config.max_det << " target_class=" << config.target_class << " max_hz=" << config.max_hz
         << " max_age_ms=" << config.max_age_ms;
    return text.str();
}

}  // namespace pure_uav::detection
