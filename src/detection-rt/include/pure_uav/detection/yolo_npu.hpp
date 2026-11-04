#pragma once
// VIPLite/awnn 上的 YOLO 推理后端；接口与 YoloCpu 一致。
// NBG 输入/输出与量化处理见同目录 README。

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "pure_uav/detection/yolo_output.hpp"

namespace pure_uav {
namespace detection {

/// awnn 推理封装；单实例，infer() 不可并发调用。
class YoloNpu {
public:
    YoloNpu();
    ~YoloNpu();

    YoloNpu(const YoloNpu&) = delete;
    YoloNpu& operator=(const YoloNpu&) = delete;

    /// 加载 .nb 模型。
    /// @param[in] model_path NBG 文件路径
    /// @param[in] imgsz 输入边长，目前固定 640
    /// @param[out] error 失败原因
    /// @return 成功为 true。
    bool load(const std::string& model_path, int imgsz, std::string& error);

    /// 执行预处理、前向和输出融合。
    /// @param[in] bgr CV_8UC3 原图
    /// @param[out] out 指向内部复用缓冲，仅在下一次 infer 前有效
    /// @param[out] error 失败原因
    /// @return 成功为 true。
    bool infer(const cv::Mat& bgr, YoloOutputView& out, std::string& error);

    bool loaded() const;
    int inputSize() const;

    /// NBG 输入/输出维度、dtype 与量化参数描述。
    const std::string& info() const;

    /// 销毁 awnn 上下文；析构时自动调用。
    void release();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

}  // namespace detection
}  // namespace pure_uav
