#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace pure_uav {

// 原生深度与原始彩色像素域的标定。畸变顺序与 OpenCV rational Brown 模型相同。
struct RangeCameraModel {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::array<double, 8> distortion{}; // k1,k2,p1,p2,k3,k4,k5,k6
    bool valid() const;
};

struct DepthRangeCalibration {
    RangeCameraModel depth, color;
    // p_color = R_depth_to_color * p_depth + t_depth_to_color_m
    cv::Matx33d R_depth_to_color = cv::Matx33d::eye();
    cv::Vec3d t_depth_to_color_m{0, 0, 0};
    bool valid() const;
};

// 读取 Kalibr cam0 彩色标定（pinhole/radtan），必须匹配实际原始彩色分辨率。
// 失败时清空 model 并返回说明；不缩放、不猜裁剪，不读取相机-IMU 外参作 D2C 外参。
bool loadRangeColorCalibration(const std::string& path, int width, int height,
                               RangeCameraModel& model, std::string* error = nullptr);

// SDK 原始 Y16 转为毫米；过滤无效码后缩放，单位无效时返回空图。
cv::Mat depthMillimeters(const cv::Mat& raw_depth, double value_scale_mm);

struct DepthRangeSample {
    bool has_range = false;
    double range_m = 0.0; // 彩色相机中心到所查询像素表面的直线距离，不是光轴深度 Z
};

// 两路设备时间同源：把深度时间换到彩色帧已有的时钟域；未知/下溢/溢出返回 0。
uint64_t depthTimestampFromColor(uint64_t color_timestamp_us, int64_t color_device_us,
                                 int64_t depth_device_us);

// 一帧深度投影一次，可供同帧多个检测框查询。实例由一个处理线程持有。
// 与 IMU、VIO 初始化及飞控是否连接无关；两个时间戳必须在同一个时钟域。
class DepthRangeProjector {
public:
    struct Options {
        double min_depth_m = 0.15;
        double max_depth_m = 10.0;
        int patch_radius = 4;          // 彩色像素域 9x9，容纳约 3 倍焦距比下至少 5 个深度采样点
        int min_valid_pixels = 5;
        double max_spread_m = 0.10;     // 两个表面混在一起时拒绝输出
        double max_spread_fraction = 0.05;
        uint64_t max_time_delta_us = 50000;
    };

    DepthRangeProjector();
    explicit DepthRangeProjector(const Options& options);
    bool configure(const DepthRangeCalibration& calibration);
    bool configured() const { return configured_; }

    // 输入 CV_16UC1 原生深度，单位毫米（0/65535 无效）。输入须与 depth 标定尺寸一致。
    // 每次调用先清掉旧帧；空图、错误格式/尺寸/时间戳都不会留下上次测距。
    bool update(const cv::Mat& depth_mm, uint64_t depth_timestamp_us);
    void clear();
    DepthRangeSample sample(double color_u_px, double color_v_px, uint64_t color_timestamp_us) const;

private:
    Options options_;
    DepthRangeCalibration calibration_;
    bool configured_ = false;
    uint64_t timestamp_us_ = 0;
    cv::Mat color_z_m_;
    std::vector<cv::Point2f> depth_rays_;
    std::vector<cv::Point3f> color_points_;
    std::vector<cv::Point2f> color_pixels_;
};

} // namespace pure_uav
