#pragma once
// 编译期推理后端别名：YoloBackend = YoloNpu 或 YoloCpu。
// 后端由 CMake 的 PURE_DETECTION_BACKEND 决定。

#ifdef PURE_DETECTION_BACKEND_CPU
#include "pure_uav/detection/yolo_cpu.hpp"

namespace pure_uav::detection {
using YoloBackend = YoloCpu;
}  // namespace pure_uav::detection

#elif defined(PURE_DETECTION_BACKEND_NPU)
#include "pure_uav/detection/yolo_npu.hpp"
namespace pure_uav {
namespace detection {
using YoloBackend = YoloNpu;
}  // namespace detection
}  // namespace pure_uav
#else
#error "未定义推理后端：请通过 CMake 设置 PURE_DETECTION_BACKEND=NPU 或 CPU"
#endif
