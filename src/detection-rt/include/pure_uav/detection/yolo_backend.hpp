#pragma once
// =============================================================================
//  编译期推理后端别名
//
//  CMake 根据 PURE_DETECTION_BACKEND 定义 PURE_DETECTION_BACKEND_NPU 或
//  PURE_DETECTION_BACKEND_CPU，Detector 与 CLI 只认 YoloBackend 这一个类型。
//  两个后端都有 load()/infer()/loaded()/info()/release()，接口保持一致。
// =============================================================================

#if defined(PURE_DETECTION_BACKEND_CPU)
#include "pure_uav/detection/yolo_cpu.hpp"
namespace pure_uav {
namespace detection {
using YoloBackend = YoloCpu;
}  // namespace detection
}  // namespace pure_uav
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
