#pragma once
// =============================================================================
//  ONNX Runtime CPU 推理后端
//
//  · 用于没有 NPU（/dev/vipcore / VIPLite）的机器；接口与 YoloNpu 完全一致。
//  · 输入 1×3×640×640 FP32，RGB，[0,1]；输出 [1,6,8400]（channel-major）。
//  · 预处理与 NPU 后端保持一致：letterbox 左上角填充、BGR→RGB、NCHW float。
//  · 单实例、单线程推理；ONNX Runtime 的 intra-op 线程数默认取 8。
// =============================================================================

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "pure_uav/detection/yolo_output.hpp"

namespace pure_uav {
namespace detection {

class YoloCpu {
public:
    YoloCpu();
    ~YoloCpu();

    YoloCpu(const YoloCpu&) = delete;
    YoloCpu& operator=(const YoloCpu&) = delete;

    // model_path: yolo26n.onnx；imgsz: 640。失败返回 false 并给出可读 error。
    bool load(const std::string& model_path, int imgsz, std::string& error);
    // 预处理 + CPU 前向；out 指向内部复用缓冲，下一次 infer 前有效。
    bool infer(const cv::Mat& bgr, YoloOutputView& out, std::string& error);

    bool loaded() const;
    int inputSize() const;
    const std::string& info() const;

    void release();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detection
}  // namespace pure_uav
