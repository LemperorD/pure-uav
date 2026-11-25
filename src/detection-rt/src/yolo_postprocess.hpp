#pragma once
// 检测后处理：解码 channel-major / anchor-major 两种输出布局 + 逐类 NMS。
// 内部头文件（不对外安装），只被 detection_rt 使用。

#include "detection_types.hpp"
#include "yolo_output.hpp"

namespace pure_uav::detection {

struct PostprocessParams {
    int num_classes = 2;
    float conf_thr = 0.25F;
    float iou_thr = 0.45F;
    int max_det = 300;
    int img_w = 0;
    int img_h = 0;
};

// 把 NPU 输出写入 FrameResult 的 boxes/count/present 与主目标字段。
// 返回 false 表示输出布局无法识别（此时 count=0）。
bool postprocess(const YoloOutputView& out, const PostprocessParams& p, FrameResult& r);

}  // namespace pure_uav::detection
