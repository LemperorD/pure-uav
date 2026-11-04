#pragma once
// =============================================================================
//  推理后端（NPU / CPU）的统一输出视图
//
//  后端只负责把网络原始输出交给后处理；解码 + NMS 在 yolo_postprocess 里共用。
//  data 指向后端内部复用缓冲，只在下一次 infer 前有效。
// =============================================================================

namespace pure_uav {
namespace detection {

struct YoloOutputView {
    const float* data = nullptr;
    int channels = 0;            // 4 + nc
    int anchors = 0;
    bool channel_major = true;   // true: [1,C,A]；false: [1,A,C]
    float letterbox_scale = 1.0f;
    float infer_ms = 0.0f;       // 前向 + 输出整理耗时
};

}  // namespace detection
}  // namespace pure_uav
