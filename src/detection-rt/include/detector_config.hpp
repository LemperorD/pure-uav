#pragma once
// detect_rt 检测算法参数配置接口。
// config/detector-rt.yaml 是检测参数的唯一真源；本头文件只暴露 RuntimeConfig
// 与字符串，不把 yaml-cpp 类型泄漏给使用者。
#include <string>

#include "detection_runtime.hpp"

namespace pure_uav::detection {

/// 定位配置文件：cwd 优先，其次从 /proc/self/exe 上溯仓库根；找不到时返回原路径。
std::string locateDetectorConfig(const std::string& path);

/// model_path 为 "auto" 或空时，按 PURE_DETECTION_BACKEND_CPU/NPU 解析默认模型；
/// 相对路径按「cwd → 可执行文件上溯仓库根」定位，绝对路径原样返回。
std::string resolveModelPath(const std::string& configured);

/// 解析并严格校验配置文件；失败时 error 给出键名与原因。
/// 成功时 out 为完整可用的 RuntimeConfig（model_path 已解析为实际路径）。
bool loadDetectorConfig(const std::string& path, RuntimeConfig& out, std::string& error);

/// 供 --check-config 与启动日志输出配置摘要（单行）。
std::string describeDetectorConfig(const RuntimeConfig& config);

/// 默认配置文件路径。
constexpr const char* kDefaultDetectorConfig = "config/detector-rt.yaml";

}  // namespace pure_uav::detection
