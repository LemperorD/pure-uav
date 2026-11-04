#pragma once
// 检测模块的进程内结果类型；时间戳约定见 assets/docs/interfaces.md §4。
// FrameResult 固定容量，poll() 路径零堆分配。

#include <cstdint>

namespace pure_uav {
namespace detection {

constexpr int kMaxDetections = 300;

/// 单个检测框（原图像素坐标）。
struct DetectionBox {
    float x1, y1, x2, y2;
    float conf;             // 0~1
    int   class_id;         // 0=armor_blue, 1=armor_red
};

/// 一帧检测结果；主目标是置信度最高的框。
/// @note host_us 是帧到达主机的 Unix µs，不是曝光中点，也不是飞控 boot 时钟。
struct FrameResult {
    uint64_t host_us;       // 帧到达主机的 Unix µs
    uint64_t device_us;     // 相机设备时间戳（仅诊断）
    uint64_t seq;           // 相机帧序号，单调递增
    int      width, height; // 原始图像尺寸
    int      count;         // 有效框数量
    DetectionBox boxes[kMaxDetections];
    bool     present;       // count > 0

    // 主目标（置信度最高的框），对应 GuidanceTargetSample 的必填字段
    double u_px, v_px, w_px, h_px;
    double conf;
    int    class_id;
    float  infer_ms;        // 当前推理后端的前向耗时
};

}  // namespace detection
}  // namespace pure_uav
