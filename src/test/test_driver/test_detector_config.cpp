// detector_config 无硬件回归：配置解析、严格白名单、范围校验、模型路径解析与摘要输出。
// 不加载模型、不打开相机、不连接共享内存。
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "detector_config.hpp"

namespace {
using pure_uav::detection::RuntimeConfig;

int g_failures = 0;
std::vector<std::string> g_temp_files;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

std::string suffix() {
    return std::to_string(static_cast<long long>(getpid()));
}

std::string tempPath(const std::string& tag, const std::string& ext) {
    const auto path = std::filesystem::temp_directory_path() / ("pure_uav_det_cfg_" + suffix() + "_" + tag + ext);
    const std::string result = path.string();
    g_temp_files.push_back(result);
    return result;
}

std::string writeConfig(const std::string& tag, const std::string& body) {
    const std::string path = tempPath(tag, ".yaml");
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << body;
    return path;
}

std::string validBody(const std::string& model_path) {
    std::ostringstream text;
    text << "detection-rt:\n"
         << "  model_path: \"" << model_path << "\"\n"
         << "  imgsz: 640\n"
         << "  num_classes: 2\n"
         << "  conf_thr: 0.25\n"
         << "  iou_thr: 0.45\n"
         << "  max_det: 100\n"
         << "  target_class: -1\n"
         << "  max_hz: 10\n"
         << "  max_age_ms: 200\n";
    return text.str();
}

bool load(const std::string& body, RuntimeConfig& config, std::string& error) {
    return pure_uav::detection::loadDetectorConfig(writeConfig("case", body), config, error);
}

void testValidConfig(const std::string& dummy_model) {
    RuntimeConfig config;
    std::string error;
    CHECK(load(validBody(dummy_model), config, error));
    CHECK(error.empty());
    CHECK(config.model_path == dummy_model);
    CHECK(config.imgsz == 640);
    CHECK(config.num_classes == 2);
    CHECK(config.conf_thr > 0.24F && config.conf_thr < 0.26F);
    CHECK(config.iou_thr > 0.44F && config.iou_thr < 0.46F);
    CHECK(config.max_det == 100);
    CHECK(config.target_class == -1);
    CHECK(config.max_hz == 10);
    CHECK(config.max_age_ms == 200);

    const std::string description = pure_uav::detection::describeDetectorConfig(config);
    CHECK(description.find("imgsz=640") != std::string::npos);
    CHECK(description.find("max_hz=10") != std::string::npos);
    CHECK(description.find("target_class=-1") != std::string::npos);
    CHECK(description.find(dummy_model) != std::string::npos);
}

void testNonDefaultValues(const std::string& dummy_model) {
    std::string body = validBody(dummy_model);
    const auto replace = [&body](const std::string& from, const std::string& to) {
        const auto at = body.find(from);
        CHECK(at != std::string::npos);
        if (at != std::string::npos) {
            body.replace(at, from.size(), to);
        }
    };
    replace("conf_thr: 0.25", "conf_thr: 0.7");
    replace("target_class: -1", "target_class: 1");
    replace("max_hz: 10", "max_hz: 25");
    replace("max_age_ms: 200", "max_age_ms: 750");
    RuntimeConfig config;
    std::string error;
    CHECK(load(body, config, error));
    CHECK(config.conf_thr > 0.69F && config.conf_thr < 0.71F);
    CHECK(config.target_class == 1);
    CHECK(config.max_hz == 25);
    CHECK(config.max_age_ms == 750);
}

void testAutoModel() {
    RuntimeConfig config;
    std::string error;
    CHECK(load(validBody("auto"), config, error));
    CHECK(config.model_path.find("yolo26n") != std::string::npos);
    CHECK(std::filesystem::is_regular_file(config.model_path));

    const std::string resolved = pure_uav::detection::resolveModelPath("auto");
    CHECK(resolved == config.model_path);
    CHECK(pure_uav::detection::resolveModelPath("") == resolved);
    CHECK(pure_uav::detection::resolveModelPath("/tmp/explicit_model.nb") == "/tmp/explicit_model.nb");
}

void testRejectedBodies(const std::string& dummy_model) {
    std::string bad_type = validBody(dummy_model);
    const auto type_at = bad_type.find("conf_thr: 0.25");
    CHECK(type_at != std::string::npos);
    bad_type.replace(type_at, std::string("conf_thr: 0.25").size(), "conf_thr: text");

    struct Case {
        std::string tag;
        std::string body;
        std::string expected;
    };
    const std::vector<Case> cases = {
        {"unknown_key", validBody(dummy_model) + "  surprise: 1\n", "未知字段"}, {"legacy_conf", "detection-rt:\n  model: yolo26n\n  confidence_threshold: 0.5\n", "旧占位字段"},
        {"legacy_device", "detection-rt:\n  device: cuda\n", "旧占位字段"},      {"legacy_input_size", "detection-rt:\n  input_size: [640, 640]\n", "旧占位字段"},
        {"legacy_iou", "detection-rt:\n  iou_threshold: 0.4\n", "旧占位字段"},   {"missing_top_key", "detection:\n  imgsz: 640\n", "缺少 detection-rt"},
        {"missing_field", "detection-rt:\n  imgsz: 640\n", "缺少必填字段"},      {"bad_type", bad_type, "取值无效"},
    };
    for (const auto& item : cases) {
        RuntimeConfig config;
        std::string error;
        const bool ok = pure_uav::detection::loadDetectorConfig(writeConfig(item.tag, item.body), config, error);
        CHECK(!ok);
        CHECK(!error.empty());
        if (error.find(item.expected) == std::string::npos) {
            std::fprintf(stderr, "FAIL %s: expected '%s', got '%s'\n", item.tag.c_str(), item.expected.c_str(), error.c_str());
            ++g_failures;
        }
    }
}

void testRangeRejections(const std::string& dummy_model) {
    struct Case {
        const char* field;
        const char* bad_value;
    };
    const std::vector<Case> cases = {
        {"imgsz: 640", "imgsz: 320"}, {"conf_thr: 0.25", "conf_thr: 1.5"}, {"iou_thr: 0.45", "iou_thr: 0"}, {"max_det: 100", "max_det: 301"}, {"target_class: -1", "target_class: 9"}, {"max_hz: 10", "max_hz: 0"}, {"max_age_ms: 200", "max_age_ms: 5000"}, {"num_classes: 2", "num_classes: 0"},
    };
    for (const auto& item : cases) {
        std::string body = validBody(dummy_model);
        const auto at = body.find(item.field);
        CHECK(at != std::string::npos);
        if (at == std::string::npos) {
            continue;
        }
        body.replace(at, std::string(item.field).size(), item.bad_value);
        RuntimeConfig config;
        std::string error;
        CHECK(!load(body, config, error));
        CHECK(!error.empty());
    }
}

void testMissingFiles() {
    RuntimeConfig config;
    std::string error;
    CHECK(!load(validBody("/tmp/pure_uav_no_such_model_file.onnx"), config, error));
    CHECK(error.find("模型文件不存在") != std::string::npos);

    CHECK(!pure_uav::detection::loadDetectorConfig("/tmp/pure_uav_no_such_detector_config.yaml", config, error));
    CHECK(error.find("找不到检测配置文件") != std::string::npos);

    const std::string empty = writeConfig("empty", "");
    CHECK(!pure_uav::detection::loadDetectorConfig(empty, config, error));
    CHECK(!error.empty());
}

void testLocateConfig() {
    const std::string path = pure_uav::detection::locateDetectorConfig(pure_uav::detection::kDefaultDetectorConfig);
    CHECK(std::filesystem::is_regular_file(path));
    RuntimeConfig config;
    std::string error;
    CHECK(pure_uav::detection::loadDetectorConfig(path, config, error));
    CHECK(config.imgsz == 640);
    CHECK(config.max_hz == 10);
}

}  // namespace

int main() {
    const std::string dummy_model = tempPath("model", ".onnx");
    {
        std::ofstream out(dummy_model, std::ios::binary | std::ios::trunc);
        out << "dummy model for config test\n";
    }

    testValidConfig(dummy_model);
    testNonDefaultValues(dummy_model);
    testAutoModel();
    testRejectedBodies(dummy_model);
    testRangeRejections(dummy_model);
    testMissingFiles();
    testLocateConfig();

    for (const auto& path : g_temp_files) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    if (g_failures != 0) {
        std::fprintf(stderr, "test_detector_config: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_detector_config: OK\n");
    return 0;
}
