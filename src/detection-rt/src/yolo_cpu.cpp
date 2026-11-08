// ONNX Runtime CPU 后端实现；预处理与 NPU 后端保持一致。
#include "pure_uav/detection/yolo_cpu.hpp"

#include <onnxruntime_cxx_api.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>

namespace pure_uav::detection {
namespace {

double msSince(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

std::string shapeStr(const std::vector<int64_t>& s) {
    std::ostringstream os;
    for (size_t i = 0; i < s.size(); ++i) {
        if (i != 0U) {
            os << "x";
        }
        os << s[i];
    }
    return os.str();
}

const std::string& emptyString() {
    static const std::string kEmpty;
    return kEmpty;
}

}  // namespace

struct YoloCpu::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "detection_rt"};
    std::unique_ptr<Ort::Session> session;

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;

    int imgsz = 0;
    int threads = 8;
    int channels = 0;
    int anchors = 0;
    bool channel_major = true;

    std::vector<float> input_blob;
    std::vector<float> output_data;
    std::string info;

    void clear() {
        session.reset();
        input_names.clear();
        output_names.clear();
        input_name_ptrs.clear();
        output_name_ptrs.clear();
        input_blob.clear();
        input_blob.shrink_to_fit();
        output_data.clear();
        output_data.shrink_to_fit();
        imgsz = 0;
        channels = anchors = 0;
        info.clear();
    }
};

YoloCpu::YoloCpu() : impl_(new Impl()) {
}
YoloCpu::~YoloCpu() {
    release();
}

bool YoloCpu::load(const std::string& model_path, int imgsz, std::string& error) {
    release();

    if (model_path.empty() || access(model_path.c_str(), R_OK) != 0) {
        error = "ONNX 模型文件不存在或不可读: " + model_path;
        return false;
    }
    if (imgsz != 640) {
        error = "CPU 后端只支持 640x640 输入，收到 imgsz=" + std::to_string(imgsz);
        return false;
    }

    try {
        Ort::SessionOptions options;
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        impl_->threads = std::min(8, std::max(1, static_cast<int>(std::thread::hardware_concurrency())));
        options.SetIntraOpNumThreads(impl_->threads);
        options.SetExecutionMode(ORT_SEQUENTIAL);
        impl_->session = std::make_unique<Ort::Session>(impl_->env, model_path.c_str(), options);

        Ort::AllocatorWithDefaultOptions allocator;
        for (size_t i = 0; i < impl_->session->GetInputCount(); ++i) {
            impl_->input_names.emplace_back(impl_->session->GetInputNameAllocated(i, allocator).get());
        }
        for (size_t i = 0; i < impl_->session->GetOutputCount(); ++i) {
            impl_->output_names.emplace_back(impl_->session->GetOutputNameAllocated(i, allocator).get());
        }
        for (const auto& s : impl_->input_names) {
            impl_->input_name_ptrs.push_back(s.c_str());
        }
        for (const auto& s : impl_->output_names) {
            impl_->output_name_ptrs.push_back(s.c_str());
        }

        if (impl_->input_name_ptrs.size() != 1) {
            error = "ONNX 模型输入数量为 " + std::to_string(impl_->input_name_ptrs.size()) + "，期望 1";
            impl_->clear();
            return false;
        }

        // 注意：TypeInfo 必须先存成局部变量，否则 GetTensorTypeAndShapeInfo()
        // 返回的对象会引用已销毁的临时 TypeInfo，读到非法元素类型。
        auto in_type_info = impl_->session->GetInputTypeInfo(0);
        auto in_info = in_type_info.GetTensorTypeAndShapeInfo();
        if (in_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            error = "CPU 后端只支持 FP32 ONNX 输入（当前元素类型 " + std::to_string(static_cast<int>(in_info.GetElementType())) + "）";
            impl_->clear();
            return false;
        }
        const auto in_shape = in_info.GetShape();
        if (in_shape.size() != 4 || in_shape[1] != 3) {
            error = "ONNX 输入维度不是 [1,3,H,W]: " + shapeStr(in_shape);
            impl_->clear();
            return false;
        }
        const int h = in_shape[2] > 0 ? static_cast<int>(in_shape[2]) : imgsz;
        const int w = in_shape[3] > 0 ? static_cast<int>(in_shape[3]) : imgsz;
        if (h != imgsz || w != imgsz) {
            error = "ONNX 输入 " + std::to_string(w) + "x" + std::to_string(h) + " 与 imgsz=" + std::to_string(imgsz) + " 不符";
            impl_->clear();
            return false;
        }
        impl_->imgsz = imgsz;

        if (impl_->session->GetOutputCount() < 1) {
            error = "ONNX 模型没有输出";
            impl_->clear();
            return false;
        }
        auto out_type_info = impl_->session->GetOutputTypeInfo(0);
        auto out_info = out_type_info.GetTensorTypeAndShapeInfo();
        if (out_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            error = "ONNX 输出不是 FP32";
            impl_->clear();
            return false;
        }
        auto out_shape = out_info.GetShape();
        while (out_shape.size() > 2 && out_shape.front() == 1) {
            out_shape.erase(out_shape.begin());
        }
        if (out_shape.size() != 2) {
            error = "ONNX 输出维度不是 [1,C,A] 或 [1,A,C]: " + shapeStr(out_info.GetShape());
            impl_->clear();
            return false;
        }
        const int d0 = static_cast<int>(out_shape[0]);
        const int d1 = static_cast<int>(out_shape[1]);
        impl_->channel_major = (d1 > d0);  // anchors 远大于通道数
        impl_->channels = std::min(d0, d1);
        impl_->anchors = std::max(d0, d1);
        if (impl_->channels < 4 || impl_->anchors <= 0) {
            error = "ONNX 输出布局无法识别: " + shapeStr(out_info.GetShape());
            impl_->clear();
            return false;
        }

        impl_->input_blob.assign(static_cast<size_t>(3) * imgsz * imgsz, 0.0F);

        std::ostringstream os;
        os << "ONNX Runtime CPU; input " << shapeStr(in_shape) << " FP32; output " << (impl_->channel_major ? "CxA " : "AxC ") << impl_->channels << "x" << impl_->anchors << "; threads=" << impl_->threads;
        impl_->info = os.str();
        return true;
    } catch (const Ort::Exception& e) {
        error = std::string("ONNX Runtime 加载失败: ") + e.what();
        impl_->clear();
        return false;
    } catch (const std::exception& e) {
        error = std::string("ONNX Runtime 加载异常: ") + e.what();
        impl_->clear();
        return false;
    }
}

bool YoloCpu::infer(const cv::Mat& bgr, YoloOutputView& out, std::string& error) {
    out = YoloOutputView();
    if (!impl_ || !impl_->session) {
        error = "YoloCpu 未加载模型";
        return false;
    }
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        error = "YoloCpu::infer 需要非空 CV_8UC3 BGR 图";
        return false;
    }

    const int H = impl_->imgsz;
    const int W = impl_->imgsz;

    // letterbox：与 NPU 后端一致，左上角填充、零填充、BGR→RGB
    const float scale = std::min(static_cast<float>(W) / static_cast<float>(bgr.cols), static_cast<float>(H) / static_cast<float>(bgr.rows));
    int new_w = static_cast<int>(std::round(bgr.cols * scale));
    int new_h = static_cast<int>(std::round(bgr.rows * scale));
    new_w = std::min(new_w, W);
    new_h = std::min(new_h, H);
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat canvas = cv::Mat::zeros(H, W, CV_8UC3);
    resized.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    // NCHW FP32 [0,1]
    const int area = H * W;
    for (int y = 0; y < H; ++y) {
        const cv::Vec3b* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) {
            const cv::Vec3b& px = row[x];
            const auto idx = static_cast<size_t>((y * W) + x);
            impl_->input_blob[idx] = static_cast<float>(px[0]) / 255.0F;
            impl_->input_blob[area + idx] = static_cast<float>(px[1]) / 255.0F;
            impl_->input_blob[(2 * area) + idx] = static_cast<float>(px[2]) / 255.0F;
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    try {
        const std::array<int64_t, 4> input_shape{1, 3, H, W};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, impl_->input_blob.data(), impl_->input_blob.size(), input_shape.data(), input_shape.size());

        auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, impl_->input_name_ptrs.data(), &input_tensor, 1, impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());
        if (outputs.empty()) {
            error = "ONNX Runtime 没有返回输出";
            return false;
        }

        auto info = outputs[0].GetTensorTypeAndShapeInfo();
        const size_t count = info.GetElementCount();
        if (count != static_cast<size_t>(impl_->channels) * impl_->anchors) {
            error = "ONNX 输出元素数 " + std::to_string(count) + " 与 " + std::to_string(impl_->channels) + "x" + std::to_string(impl_->anchors) + " 不符";
            return false;
        }
        const auto* data = outputs[0].GetTensorData<float>();
        impl_->output_data.assign(data, data + count);

        out.data = impl_->output_data.data();
        out.channels = impl_->channels;
        out.anchors = impl_->anchors;
        out.channel_major = impl_->channel_major;
        out.letterbox_scale = scale;
        out.infer_ms = static_cast<float>(msSince(t0));
        return true;
    } catch (const Ort::Exception& e) {
        error = std::string("ONNX Runtime 推理失败: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        error = std::string("ONNX Runtime 推理异常: ") + e.what();
        return false;
    }
}

bool YoloCpu::loaded() const {
    return impl_ && impl_->session != nullptr;
}

int YoloCpu::inputSize() const {
    return impl_ ? impl_->imgsz : 0;
}

const std::string& YoloCpu::info() const {
    return impl_ ? impl_->info : emptyString();
}

void YoloCpu::release() {
    if (impl_) {
        impl_->clear();
    }
}

}  // namespace pure_uav::detection
