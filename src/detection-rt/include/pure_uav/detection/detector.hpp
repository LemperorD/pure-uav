#pragma once
// 检测门面：采集线程发布最新 BGR 帧，推理线程发布最新 FrameResult。
// poll() 非阻塞且只返回最新结果；线程模型见同目录 README。

#include <cstdint>
#include <memory>
#include <string>

#include "pure_uav/detection/camera_source.hpp"
#include "pure_uav/detection/detection_types.hpp"

namespace pure_uav {
namespace detection {

/// 检测器配置；model_path 为空时由调用方负责给出有效模型路径。
struct DetectorConfig {
    CameraConfig camera;

    std::string model_path;
    int imgsz = 640;
    int num_classes = 2;
    float conf_thr = 0.25f;
    float iou_thr = 0.45f;
    int max_det = 300;

    bool verbose = false;
};

/// 启动以来的采集/处理统计。
struct DetectorStats {
    double capture_fps = 0.0;
    double process_fps = 0.0;
    double avg_infer_ms = 0.0;
    uint64_t capture_frames = 0;
    uint64_t process_frames = 0;
};

/// 检测门面；同一进程只允许一个实例（推理后端是进程级资源）。
class Detector {
public:
    Detector();
    ~Detector();

    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    /// 加载推理后端并启动相机与推理线程。
    /// @param[out] error 失败原因
    /// @return 成功为 true。
    bool start(const DetectorConfig& cfg, std::string& error);

    /// 停止采集/推理线程并释放相机与推理后端；可重复调用。
    void stop();

    bool running() const;
    bool alive() const;

    /// 取最新检测结果。
    /// @param[out] out 有新结果时覆盖；返回 false 时不修改
    /// @return 自上次 poll 后是否有新结果
    /// @note 非阻塞，可在定周期循环中每拍调用。
    bool poll(FrameResult& out);

    /// 启动以来采集/处理的平均统计。
    DetectorStats stats() const;

    /// 最近一次已发布结果对应的 BGR 帧（浅拷贝），仅供可视化。
    bool latest_frame(cv::Mat& out) const;

    /// 相机实际打开的 profile（"WxH@FPS fmtF"）。
    std::string cameraProfile() const;

    /// 当前推理后端的输入/输出与配置信息。
    std::string modelInfo() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detection
}  // namespace pure_uav
