#include "detection_runtime.hpp"

#if PURE_UAV_HAS_NPU
#include "yolo_npu.hpp"
#include "yolo_postprocess.hpp"

namespace pure_uav::detection {
namespace {
class NpuBackend final : public InferenceBackend {
  public:
    bool load(const RuntimeConfig& config, std::string& error) override {
        config_ = config;
        return npu_.load(config.model_path, config.imgsz, error);
    }
    bool infer(const cv::Mat& bgr, FrameResult& result, std::string& error) override {
        NpuOutputView output;
        if (!npu_.infer(bgr, output, error)) return false;
        if (output.anchors != 8400) {
            error = "Expected 8400 decoded YOLO anchors for the existing 640x640 model; end-to-end output is unsupported";
            return false;
        }
        PostprocessParams params;
        params.num_classes = config_.num_classes;
        params.conf_thr = config_.conf_thr;
        params.iou_thr = config_.iou_thr;
        params.max_det = config_.max_det;
        params.img_w = bgr.cols;
        params.img_h = bgr.rows;
        result.infer_ms = output.infer_ms;
        if (!postprocess(output, params, result)) {
            error = "Unsupported model output: expected decoded YOLO xywh + configured class probabilities";
            return false;
        }
        // 与 CPU 适配层一致：后处理无类别过滤，需在此满足 validResult() 约束。
        if (config_.target_class >= 0) {
            int kept = 0;
            for (int index = 0; index < result.count; ++index) {
                if (result.boxes[index].class_id == config_.target_class) {
                    if (kept != index) {
                        result.boxes[kept] = result.boxes[index];
                    }
                    ++kept;
                }
            }
            result.count = kept;
            result.present = kept > 0;
        }
        return true;
    }
    void close() noexcept override {
        npu_.release();
    }

  private:
    RuntimeConfig config_;
    YoloNpu npu_;
};
}  // namespace
bool npuBackendAvailable() {
    return true;
}
std::unique_ptr<InferenceBackend> makeNpuBackend() {
    return std::make_unique<NpuBackend>();
}
}  // namespace pure_uav::detection
#else
namespace pure_uav::detection {
bool npuBackendAvailable() {
    return false;
}
std::unique_ptr<InferenceBackend> makeNpuBackend() {
    return {};
}
}  // namespace pure_uav::detection
#endif
