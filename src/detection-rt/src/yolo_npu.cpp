#include "pure_uav/detection/yolo_npu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

#include <vip_lite.h>

extern "C" {
#include "awnn_lib.h"
#include "awnn_internal.h"
}

namespace pure_uav {
namespace detection {
namespace {

constexpr int kMaxCreateAttempts = 4;  // 首次 + 最多 3 次重试

const char* formatName(int f) {
    switch (f) {
        case VIP_BUFFER_FORMAT_FP32: return "FP32";
        case VIP_BUFFER_FORMAT_FP16: return "FP16";
        case VIP_BUFFER_FORMAT_UINT8: return "UINT8";
        case VIP_BUFFER_FORMAT_INT8: return "INT8";
        case VIP_BUFFER_FORMAT_INT16: return "INT16";
        default: return "OTHER";
    }
}

const char* quantName(int q) {
    switch (q) {
        case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT: return "DFP";
        case VIP_BUFFER_QUANTIZE_TF_ASYMM: return "TF_ASYMM";
        case VIP_BUFFER_QUANTIZE_NONE: return "NONE";
        default: return "OTHER";
    }
}

std::string dimsStr(const vip_buffer_create_params_t& p) {
    std::ostringstream os;
    for (int i = 0; i < p.num_of_dims; ++i) {
        if (i) os << "x";
        os << p.sizes[i];
    }
    return os.str();
}

double msSince(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t)
        .count();
}

const std::string& emptyString() {
    static const std::string kEmpty;
    return kEmpty;
}

}  // namespace

struct YoloNpu::Impl {
    Awnn_Context_t* ctx = nullptr;
    bool initialized = false;
    int imgsz = 0;

    // 输入描述
    bool nhwc = false;
    int width = 0;
    int height = 0;
    int input_dtype = VIP_BUFFER_FORMAT_UINT8;
    int input_quant = VIP_BUFFER_QUANTIZE_NONE;
    float input_scale = 1.0f;
    int input_zero = 0;
    int input_dfp = 0;
    std::array<uint8_t, 256> quant_lut{};
    std::vector<uint8_t> input_bytes;

    // 输出描述
    int out_channels = 0;
    int out_anchors = 0;
    bool channel_major = true;
    std::vector<int64_t> out_shape;
    size_t out_elements = 0;
    int fallback_index = 0;
    std::vector<float> fused;
    std::string info;

    void clear() {
        if (ctx) {
            awnn_destroy(ctx);
            ctx = nullptr;
        }
        if (initialized) {
            awnn_uninit();
            initialized = false;
        }
        input_bytes.clear();
        input_bytes.shrink_to_fit();
        fused.clear();
        fused.shrink_to_fit();
        out_shape.clear();
        out_elements = 0;
        out_channels = out_anchors = 0;
        width = height = imgsz = 0;
        info.clear();
    }

    void buildQuantLut() {
        const bool is_int8 = (input_dtype == VIP_BUFFER_FORMAT_INT8);
        for (int p = 0; p < 256; ++p) {
            const float v = static_cast<float>(p) / 255.0f;
            float qf = v * 255.0f;
            if (input_quant == VIP_BUFFER_QUANTIZE_TF_ASYMM && input_scale > 0.0f) {
                qf = v / input_scale + static_cast<float>(input_zero);
            } else if (input_quant == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT) {
                qf = v * static_cast<float>(1 << input_dfp);
            }
            int q = static_cast<int>(std::round(qf));
            q = std::max(is_int8 ? -128 : 0, std::min(is_int8 ? 127 : 255, q));
            quant_lut[static_cast<size_t>(p)] = static_cast<uint8_t>(static_cast<int8_t>(q));
        }
    }

    void describeOutputs(std::ostringstream& os) {
        os << "; outputs=" << ctx->output_count;
        for (int i = 0; i < static_cast<int>(ctx->output_count); ++i) {
            const Awnn_params_t& op = ctx->output_params[i];
            std::vector<int64_t> shape;
            for (int d = 0; d < op.vip_param.num_of_dims; ++d) shape.push_back(op.vip_param.sizes[d]);
            os << " [" << i << "]";
            for (size_t k = 0; k < shape.size(); ++k) os << (k ? "x" : "") << shape[k];
            os << " fmt=" << formatName(op.vip_param.data_format)
               << " quant=" << quantName(op.vip_param.quant_format) << " n=" << op.elements;
        }
    }

    // 输出融合：[4,8400] + [2,8400] → channel-major [6,8400]；否则回退最大输出。
    void resolveOutputs() {
        fused.clear();
        if (ctx->output_count == 2 && ctx->output_params[0].elements % 4 == 0) {
            const size_t anchors = ctx->output_params[0].elements / 4;
            if (ctx->output_params[1].elements == anchors * 2) {
                const size_t channels = 6;
                fused.resize(channels * anchors);
                out_channels = static_cast<int>(channels);
                out_anchors = static_cast<int>(anchors);
                channel_major = true;
                out_shape = {1, static_cast<int64_t>(channels), static_cast<int64_t>(anchors)};
                out_elements = channels * anchors;
                fallback_index = -1;
                return;
            }
        }

        int best = 0;
        size_t best_elems = 0;
        for (int i = 0; i < static_cast<int>(ctx->output_count); ++i) {
            const Awnn_params_t& op = ctx->output_params[i];
            if (op.vip_param.num_of_dims >= 2 && op.elements > best_elems) {
                best_elems = op.elements;
                best = i;
            }
        }
        fallback_index = best;
        out_elements = best_elems;
        out_shape.clear();
        const Awnn_params_t& op = ctx->output_params[best];
        for (int d = 0; d < op.vip_param.num_of_dims; ++d) out_shape.push_back(op.vip_param.sizes[d]);
        while (out_shape.size() > 2 && out_shape.front() == 1) out_shape.erase(out_shape.begin());
        while (out_shape.size() > 2 && out_shape.back() == 1) out_shape.pop_back();
        if (out_shape.size() == 2) {
            channel_major = out_shape[0] <= out_shape[1];
            out_channels = static_cast<int>(channel_major ? out_shape[0] : out_shape[1]);
            out_anchors = static_cast<int>(channel_major ? out_shape[1] : out_shape[0]);
        } else {
            channel_major = true;
            out_channels = 0;
            out_anchors = 0;
        }
    }
};

YoloNpu::YoloNpu() : impl_(new Impl()) {}

YoloNpu::~YoloNpu() {
    release();
    delete impl_;
    impl_ = nullptr;
}

bool YoloNpu::load(const std::string& model_path, int imgsz, std::string& error) {
    release();

    if (model_path.empty() || access(model_path.c_str(), R_OK) != 0) {
        error = "模型文件不存在或不可读: " + model_path;
        return false;
    }
    if (imgsz != 640) {
        error = "本模块只支持 640x640 输入，收到 imgsz=" + std::to_string(imgsz);
        return false;
    }
    impl_->imgsz = imgsz;

    // awnn_init 每进程一次；create 失败时按 500 ms 间隔重试（最多 3 次重试）。
    awnn_init();
    impl_->initialized = true;

    for (int attempt = 1; attempt <= kMaxCreateAttempts; ++attempt) {
        impl_->ctx = awnn_create(model_path.c_str());
        if (impl_->ctx) break;
        if (attempt == kMaxCreateAttempts) break;
        std::cerr << "[npu] awnn_create attempt " << attempt << "/" << kMaxCreateAttempts
                  << " failed, retrying in 500 ms...\n";
        awnn_uninit();
        impl_->initialized = false;
        usleep(500000);
        awnn_init();
        impl_->initialized = true;
    }
    if (!impl_->ctx) {
        error = "awnn_create 失败（NBG 与本机 NPU 不兼容或文件损坏）: " + model_path;
        impl_->clear();
        return false;
    }

    if (impl_->ctx->input_count != 1) {
        error = "NBG 输入数量为 " + std::to_string(impl_->ctx->input_count) + "，期望 1";
        impl_->clear();
        return false;
    }

    const Awnn_params_t& in = impl_->ctx->input_params[0];
    const vip_buffer_create_params_t& ip = in.vip_param;
    impl_->input_dtype = ip.data_format;
    impl_->input_quant = ip.quant_format;

    if (ip.num_of_dims != 4) {
        error = "NBG 输入不是 4 维: " + dimsStr(ip);
        impl_->clear();
        return false;
    }

    if (ip.sizes[1] == 3) {
        // [N, C, H, W]，NCHW 内存布局
        impl_->nhwc = false;
        impl_->height = static_cast<int>(ip.sizes[2]);
        impl_->width = static_cast<int>(ip.sizes[3]);
    } else if (ip.sizes[3] == 3) {
        // [N, H, W, C]，NHWC 内存布局
        impl_->nhwc = true;
        impl_->height = static_cast<int>(ip.sizes[1]);
        impl_->width = static_cast<int>(ip.sizes[2]);
    } else if (ip.sizes[2] == 3 && ip.sizes[3] == 1) {
        // VIPLite 对 NCHW 网络输入报 [W, H, C, N]
        impl_->nhwc = false;
        impl_->width = static_cast<int>(ip.sizes[0]);
        impl_->height = static_cast<int>(ip.sizes[1]);
    } else {
        error = "不支持的 NBG 输入维度顺序: " + dimsStr(ip);
        impl_->clear();
        return false;
    }

    if (impl_->width != imgsz || impl_->height != imgsz) {
        error = "NBG 输入 " + std::to_string(impl_->width) + "x" +
                std::to_string(impl_->height) + " != " + std::to_string(imgsz);
        impl_->clear();
        return false;
    }
    if (in.elements != static_cast<size_t>(3) * imgsz * imgsz) {
        error = "NBG 输入元素数 " + std::to_string(in.elements) + " 与 3x" +
                std::to_string(imgsz) + "x" + std::to_string(imgsz) + " 不符";
        impl_->clear();
        return false;
    }

    if (impl_->input_dtype != VIP_BUFFER_FORMAT_UINT8 &&
        impl_->input_dtype != VIP_BUFFER_FORMAT_INT8 &&
        impl_->input_dtype != VIP_BUFFER_FORMAT_FP32) {
        error = std::string("不支持的 NBG 输入 dtype: ") + formatName(impl_->input_dtype);
        impl_->clear();
        return false;
    }

    // 量化参数从运行时读取
    if (impl_->input_quant == VIP_BUFFER_QUANTIZE_TF_ASYMM) {
        impl_->input_scale = ip.quant_data.affine.scale;
        impl_->input_zero = ip.quant_data.affine.zeroPoint;
        if (impl_->input_scale <= 0.0f) {
            error = "NBG 输入 TF_ASYMM scale 非法";
            impl_->clear();
            return false;
        }
        impl_->buildQuantLut();
    } else if (impl_->input_quant == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT) {
        impl_->input_dfp = ip.quant_data.dfp.fixed_point_pos;
        impl_->buildQuantLut();
    }

    const size_t bytes_per_element =
        (impl_->input_dtype == VIP_BUFFER_FORMAT_FP32) ? sizeof(float) : sizeof(uint8_t);
    impl_->input_bytes.assign(static_cast<size_t>(3) * impl_->height * impl_->width *
                                 bytes_per_element, 0);

    impl_->resolveOutputs();
    if (impl_->out_channels <= 0 || impl_->out_anchors <= 0) {
        error = "无法解析 NBG 输出布局";
        impl_->clear();
        return false;
    }

    std::ostringstream os;
    os << "VIPLite driver=0x" << std::hex << vip_get_version() << std::dec
       << "; input " << dimsStr(ip) << " fmt=" << formatName(impl_->input_dtype)
       << " quant=" << quantName(impl_->input_quant) << " scale=" << impl_->input_scale
       << " zero=" << impl_->input_zero << " dfp=" << impl_->input_dfp
       << "; fused " << (impl_->channel_major ? "channel-major " : "anchor-major ")
       << impl_->out_channels << "x" << impl_->out_anchors;
    impl_->describeOutputs(os);
    impl_->info = os.str();
    return true;
}

bool YoloNpu::infer(const cv::Mat& bgr, YoloOutputView& out, std::string& error) {
    out = YoloOutputView();
    if (!impl_ || !impl_->ctx) {
        error = "YoloNpu 未加载模型";
        return false;
    }
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        error = "YoloNpu::infer 需要非空 CV_8UC3 BGR 图";
        return false;
    }

    const int H = impl_->height;
    const int W = impl_->width;

    // letterbox：左上角填充、零填充、BGR→RGB
    const float scale =
        std::min(static_cast<float>(W) / static_cast<float>(bgr.cols),
                 static_cast<float>(H) / static_cast<float>(bgr.rows));
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

    // 打包：NCHW uint8（复用 input_bytes_）
    const int area = H * W;
    const bool fp32 = (impl_->input_dtype == VIP_BUFFER_FORMAT_FP32);
    for (int y = 0; y < H; ++y) {
        const cv::Vec3b* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) {
            const cv::Vec3b& px = row[x];
            for (int c = 0; c < 3; ++c) {
                const size_t idx = impl_->nhwc
                    ? static_cast<size_t>(y * W + x) * 3 + static_cast<size_t>(c)
                    : static_cast<size_t>(c) * area + static_cast<size_t>(y * W + x);
                if (fp32) {
                    reinterpret_cast<float*>(impl_->input_bytes.data())[idx] =
                        static_cast<float>(px[c]) / 255.0f;
                } else {
                    impl_->input_bytes[idx] = impl_->quant_lut[px[c]];
                }
            }
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    void* ptr = impl_->input_bytes.data();
    void* buffers[1] = {ptr};
    awnn_set_input_buffers(impl_->ctx, buffers);
    awnn_run(impl_->ctx);

    float** outputs = awnn_get_output_buffers(impl_->ctx);
    if (!outputs) {
        error = "awnn_get_output_buffers 返回空";
        return false;
    }

    const float* data = nullptr;
    if (impl_->fallback_index < 0) {
        // 融合后的 channel-major [6, anchors]，内部缓冲已按固定尺寸分配
        const size_t anchors = static_cast<size_t>(impl_->out_anchors);
        for (int c = 0; c < 4; ++c) {
            std::memcpy(impl_->fused.data() + static_cast<size_t>(c) * anchors,
                        outputs[0] + static_cast<size_t>(c) * anchors, anchors * sizeof(float));
        }
        for (int c = 0; c < 2; ++c) {
            std::memcpy(impl_->fused.data() + static_cast<size_t>(4 + c) * anchors,
                        outputs[1] + static_cast<size_t>(c) * anchors, anchors * sizeof(float));
        }
        data = impl_->fused.data();
    } else {
        data = outputs[impl_->fallback_index];
    }

    out.data = data;
    out.channels = impl_->out_channels;
    out.anchors = impl_->out_anchors;
    out.channel_major = impl_->channel_major;
    out.letterbox_scale = scale;
    out.infer_ms = static_cast<float>(msSince(t0));
    return true;
}

bool YoloNpu::loaded() const { return impl_ && impl_->ctx != nullptr; }

int YoloNpu::inputSize() const { return impl_ ? impl_->imgsz : 0; }

const std::string& YoloNpu::info() const { return impl_ ? impl_->info : emptyString(); }

void YoloNpu::release() {
    if (impl_) impl_->clear();
}

}  // namespace detection
}  // namespace pure_uav
