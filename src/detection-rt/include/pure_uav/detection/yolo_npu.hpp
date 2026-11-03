#pragma once
// =============================================================================
//  VIPLite/awnn 上的 YOLO26n 推理封装
//
//  · awnn_init() 每进程一次；awnn_create() 失败最多重试 3 次（间隔 500 ms，
//    重试前 awnn_uninit()/awnn_init()）。不允许每帧重新 create。
//  · 加载时校验 NBG 输入：4 维、3 通道、imgsz×imgsz，支持 [N,C,H,W] /
//    [N,H,W,C] / [W,H,C,N]；量化参数从运行时读取（TF_ASYMM 的 scale/zero 或
//    DFP 的 fixed_point_pos）。
//    BGR→RGB、NCHW uint8（按运行时量化参数打包）。
//  · 输出融合：NBG 把 [1,6,8400] 拆成 [4,8400] + [2,8400] 时拼回 channel-major
//    [6,8400]；其他布局回退到元素最多的输出。
// =============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace pure_uav {
namespace detection {

struct NpuOutputView {
    const float* data = nullptr;
    int channels = 0;        // 4 + nc
    int anchors = 0;
    bool channel_major = true;  // true: [1,C,A]；false: [1,A,C]
    float letterbox_scale = 1.0f;
    float infer_ms = 0.0f;   // NPU 前向 + 输出融合耗时
};

class YoloNpu {
public:
    YoloNpu();
    ~YoloNpu();

    YoloNpu(const YoloNpu&) = delete;
    YoloNpu& operator=(const YoloNpu&) = delete;

    // model_path: yolo26n.nb；imgsz: 640。失败返回 false 并给出可读 error。
    bool load(const std::string& model_path, int imgsz, std::string& error);
    // 执行一次预处理 + 前向 + 输出融合；out 指向内部复用缓冲，下一次 infer 前有效。
    bool infer(const cv::Mat& bgr, NpuOutputView& out, std::string& error);

    bool loaded() const;
    int inputSize() const;
    // NBG 输入/输出维度、dtype、量化参数的可读描述。
    const std::string& info() const;

    void release();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

}  // namespace detection
}  // namespace pure_uav
