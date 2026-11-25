#pragma once

#include <string>

#include "mission/mission_runtime.hpp"

namespace pure_uav::app::mission {
// 参数加载不打开设备；send_enabled 由 main 的 --no-send 决定。
bool loadRuntimeConfig(const std::string& path, RuntimeConfig& out, std::string& error);
}  // namespace pure_uav::app::mission
