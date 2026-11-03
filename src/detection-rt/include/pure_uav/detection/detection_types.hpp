#pragma once
// =============================================================================
//  检测模块的进程内结果结构（固定容量，实时使用时不分配堆内存）
//
//  【时间戳语义】
//    host_us   = 相机帧到达主机时的 Unix 微秒（system_clock），
//                **不是曝光中点，也不是飞控 boot 时钟**。
//                上层用通信模块的 SystemTime（time_unix_usec + time_boot_ms）
//                换算到飞控 boot 微秒后再填 TargetSample.boot_us。
//                曝光→到达的链路时延本次不补偿。
//    device_us = 相机设备时间戳（SDK timeStampUs()），仅作诊断，不参与计算。
// =============================================================================

#include <cstdint>

namespace pure_uav {
namespace detection {

constexpr int kMaxDetections = 300;

struct DetectionBox {
    float x1, y1, x2, y2;   // 原始彩色图像像素坐标
    float conf;             // 0~1
    int   class_id;         // 0=armor_blue, 1=armor_red
};

struct FrameResult {
    uint64_t host_us;       // 帧到达主机的 Unix 微秒
    uint64_t device_us;     // 相机设备时间戳（诊断）
    uint64_t seq;           // 相机帧序号，单调递增
    int      width, height; // 原始图像尺寸
    int      count;         // 有效框数量
    DetectionBox boxes[kMaxDetections];
    bool     present;       // count > 0

    // 主目标（置信度最高的框），直接对应 TargetSample 的必填字段
    double u_px, v_px, w_px, h_px;
    double conf;
    int    class_id;
    float  infer_ms;        // 本帧 NPU 前向 + 输出融合耗时
};

}  // namespace detection
}  // namespace pure_uav
