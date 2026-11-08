#pragma once
// ONNX Runtime CPU 推理后端；接口与 YoloNpu 一致。
// 模型输入/输出与线程配置见同目录 README。

#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "pure_uav/detection/yolo_output.hpp"

namespace pure_uav::detection {

/// CPU 推理封装；单实例，infer() 不可并发调用。
class YoloCpu {
  public:
    YoloCpu();
    ~YoloCpu();

    YoloCpu(const YoloCpu&) = delete;
    YoloCpu& operator=(const YoloCpu&) = delete;

    /// 加载 .onnx 模型。
    /// @param[in] model_path ONNX 文件路径
    /// @param[in] imgsz 输入边长，目前固定 640
    /// @param[out] error 失败原因
    /// @return 成功为 true。
    bool load(const std::string& model_path, int imgsz, std::string& error);

    /// 执行预处理和前向。
    /// @param[in] bgr CV_8UC3 原图
    /// @param[out] out 指向内部复用缓冲，仅在下一次 infer 前有效
    /// @param[out] error 失败原因
    /// @return 成功为 true。
    bool infer(const cv::Mat& image, YoloOutputView& out, std::string& error);

    bool loaded() const;
    int inputSize() const;

    /// ONNX Runtime 输入/输出与线程配置描述。
    const std::string& info() const;

    /// 释放 session；析构时自动调用。
    void release();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav::detection
