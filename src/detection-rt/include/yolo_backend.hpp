#pragma once
// 编译期推理后端别名：YoloBackend = YoloNpu 或 YoloCpu。
// 后端由 CMake 探测到的依赖决定：VIPLite 齐全用 NPU，否则用 CPU。

#ifdef PURE_DETECTION_BACKEND_CPU
#include "yolo_cpu.hpp"

namespace pure_uav::detection {
using YoloBackend = YoloCpu;
}  // namespace pure_uav::detection

#elif defined(PURE_DETECTION_BACKEND_NPU)
#include "yolo_npu.hpp"
namespace pure_uav {
namespace detection {
using YoloBackend = YoloNpu;
}  // namespace detection
}  // namespace pure_uav
#else
#error "未定义推理后端宏 PURE_DETECTION_BACKEND_NPU / PURE_DETECTION_BACKEND_CPU"
#endif
