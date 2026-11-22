#include "pure_uav/depth_range.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>

namespace pure_uav {
namespace {
cv::Matx33d intrinsics(const RangeCameraModel& camera) {
    return {camera.fx, 0, camera.cx, 0, camera.fy, camera.cy, 0, 0, 1};
}
bool validOptions(const DepthRangeProjector::Options& o) {
    const int side = 2 * std::clamp(o.patch_radius, 0, 32) + 1;
    return std::isfinite(o.min_depth_m) && std::isfinite(o.max_depth_m) && o.min_depth_m > 0 && o.max_depth_m > o.min_depth_m && o.patch_radius >= 0 && o.patch_radius <= 32 && o.min_valid_pixels > 0 && o.min_valid_pixels <= side * side && std::isfinite(o.max_spread_m) && o.max_spread_m >= 0 &&
           std::isfinite(o.max_spread_fraction) && o.max_spread_fraction >= 0;
}
}  // namespace

bool RangeCameraModel::valid() const {
    return width > 0 && height > 0 && width <= 8192 && height <= 8192 && std::isfinite(fx) && fx > 0 && std::isfinite(fy) && fy > 0 && std::isfinite(cx) && std::isfinite(cy) && std::all_of(distortion.begin(), distortion.end(), [](double d) { return std::isfinite(d); });
}

bool DepthRangeCalibration::valid() const {
    if (!depth.valid() || !color.valid()) return false;
    for (double value : R_depth_to_color.val)
        if (!std::isfinite(value)) return false;
    for (double value : t_depth_to_color_m.val)
        if (!std::isfinite(value)) return false;
    return cv::norm(cv::Mat(R_depth_to_color.t() * R_depth_to_color - cv::Matx33d::eye())) < 1e-3 && std::abs(cv::determinant(R_depth_to_color) - 1.0) < 1e-3;
}

bool loadRangeColorCalibration(const std::string& path, int width, int height, RangeCameraModel& model, std::string* error) {
    model = {};
    if (error) error->clear();
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    try {
        cv::FileStorage file(path, cv::FileStorage::READ);
        if (!file.isOpened()) return fail("彩色标定文件无法读取: " + path);
        const auto cam = file["cam0"];
        if (cam.empty()) return fail("彩色标定缺少 cam0");
        std::string camera_model, distortion_model;
        cam["camera_model"] >> camera_model;
        cam["distortion_model"] >> distortion_model;
        if (camera_model != "pinhole" || distortion_model != "radtan") return fail("测距彩色标定只支持 pinhole/radtan");
        const auto k = cam["intrinsics"], d = cam["distortion_coeffs"], size = cam["resolution"];
        if (!k.isSeq() || k.size() != 4 || !d.isSeq() || d.size() != 4 || !size.isSeq() || size.size() != 2) return fail("彩色标定的内参/畸变/分辨率字段长度错误");
        for (const auto& field : {k, d, size})
            for (const auto& value : field)
                if (!value.isInt() && !value.isReal()) return fail("彩色标定参数必须为数值");
        if (static_cast<double>(size[0]) != width || static_cast<double>(size[1]) != height) return fail("彩色标定分辨率与实际彩色流不一致；不能直接缩放或跨裁剪使用");
        RangeCameraModel candidate;
        candidate.width = width;
        candidate.height = height;
        candidate.fx = static_cast<double>(k[0]);
        candidate.fy = static_cast<double>(k[1]);
        candidate.cx = static_cast<double>(k[2]);
        candidate.cy = static_cast<double>(k[3]);
        for (int i = 0; i < 4; ++i) candidate.distortion[i] = static_cast<double>(d[i]);
        if (!candidate.valid()) return fail("彩色标定包含非有限值或无效焦距/尺寸");
        model = candidate;
        return true;
    } catch (const cv::Exception& exception) {
        return fail("彩色标定解析失败: " + std::string(exception.what()));
    }
}

cv::Mat depthMillimeters(const cv::Mat& raw_depth, double value_scale_mm) {
    if (raw_depth.empty() || raw_depth.type() != CV_16UC1 || !std::isfinite(value_scale_mm) || value_scale_mm <= 0) return {};
    cv::Mat millimeters;
    raw_depth.convertTo(millimeters, CV_16UC1, value_scale_mm);
    // 65535 是原始无效码，不能先缩放成一个普通有效深度。
    millimeters.setTo(0, (raw_depth == 0) | (raw_depth == 65535));
    return millimeters;
}

uint64_t depthTimestampFromColor(uint64_t color_timestamp_us, int64_t color_device_us, int64_t depth_device_us) {
    if (color_timestamp_us == 0 || color_device_us <= 0 || depth_device_us <= 0) return 0;
    if (depth_device_us >= color_device_us) {
        const uint64_t delta = static_cast<uint64_t>(depth_device_us - color_device_us);
        if (delta > std::numeric_limits<uint64_t>::max() - color_timestamp_us) return 0;
        return color_timestamp_us + delta;
    }
    const uint64_t delta = static_cast<uint64_t>(color_device_us - depth_device_us);
    return color_timestamp_us > delta ? color_timestamp_us - delta : 0;
}

DepthRangeProjector::DepthRangeProjector() = default;
DepthRangeProjector::DepthRangeProjector(const Options& options) : options_(options) {
}

bool DepthRangeProjector::configure(const DepthRangeCalibration& calibration) {
    configured_ = false;
    clear();
    depth_rays_.clear();
    if (!calibration.valid() || !validOptions(options_)) return false;
    calibration_ = calibration;
    std::vector<cv::Point2f> pixels;
    pixels.reserve(static_cast<size_t>(calibration.depth.width) * calibration.depth.height);
    for (int y = 0; y < calibration.depth.height; ++y)
        for (int x = 0; x < calibration.depth.width; ++x) pixels.emplace_back(x, y);
    // 深度像素的去畸变射线只在标定/分辨率变化时重算。
    cv::undistortPoints(pixels, depth_rays_, intrinsics(calibration.depth), calibration.depth.distortion);
    color_points_.reserve(pixels.size());
    color_pixels_.reserve(pixels.size());
    color_z_m_.create(calibration.color.height, calibration.color.width, CV_32FC1);
    color_z_m_.setTo(0);
    configured_ = true;
    return true;
}

void DepthRangeProjector::clear() {
    timestamp_us_ = 0;
    if (!color_z_m_.empty()) color_z_m_.setTo(0);
}

bool DepthRangeProjector::update(const cv::Mat& depth_mm, uint64_t depth_timestamp_us) {
    clear();
    if (!configured_ || depth_mm.empty() || depth_mm.type() != CV_16UC1 || depth_timestamp_us == 0 || depth_mm.cols != calibration_.depth.width || depth_mm.rows != calibration_.depth.height) return false;
    color_points_.clear();
    for (int y = 0; y < depth_mm.rows; ++y) {
        const auto* row = depth_mm.ptr<uint16_t>(y);
        for (int x = 0; x < depth_mm.cols; ++x) {
            const uint16_t millimeters = row[x];
            if (millimeters == 0 || millimeters == UINT16_MAX) continue;
            const double z = millimeters * 1e-3;
            if (z < options_.min_depth_m || z > options_.max_depth_m) continue;
            const auto ray = depth_rays_[static_cast<size_t>(y) * depth_mm.cols + x];
            const cv::Vec3d point = calibration_.R_depth_to_color * cv::Vec3d(ray.x * z, ray.y * z, z) + calibration_.t_depth_to_color_m;
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]) || point[2] <= 0) continue;
            color_points_.emplace_back(point[0], point[1], point[2]);
        }
    }
    if (color_points_.empty()) return false;
    // 正向投影避免用未知目标深度猜原生深度像素；同一彩色像素只保留最近表面。
    cv::projectPoints(color_points_, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsics(calibration_.color), calibration_.color.distortion, color_pixels_);
    for (size_t i = 0; i < color_pixels_.size(); ++i) {
        const auto pixel = color_pixels_[i];
        if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || pixel.x < 0 || pixel.y < 0 || pixel.x > color_z_m_.cols - 1 || pixel.y > color_z_m_.rows - 1) continue;
        const int x = static_cast<int>(std::lround(pixel.x)), y = static_cast<int>(std::lround(pixel.y));
        float& stored = color_z_m_.at<float>(y, x);
        const float z = color_points_[i].z;
        if (stored == 0 || z < stored) stored = z;
    }
    timestamp_us_ = depth_timestamp_us;
    return true;
}

DepthRangeSample DepthRangeProjector::sample(double u, double v, uint64_t color_timestamp_us) const {
    DepthRangeSample result;
    if (!configured_ || timestamp_us_ == 0 || color_timestamp_us == 0 || !std::isfinite(u) || !std::isfinite(v) || u < 0 || v < 0 || u > calibration_.color.width - 1 || v > calibration_.color.height - 1) return result;
    const uint64_t delta = color_timestamp_us > timestamp_us_ ? color_timestamp_us - timestamp_us_ : timestamp_us_ - color_timestamp_us;
    if (delta > options_.max_time_delta_us) return result;
    const int cx = static_cast<int>(std::lround(u)), cy = static_cast<int>(std::lround(v));
    std::vector<float> values;
    const int radius = options_.patch_radius;
    values.reserve(static_cast<size_t>(2 * radius + 1) * (2 * radius + 1));
    for (int y = std::max(0, cy - radius); y <= std::min(color_z_m_.rows - 1, cy + radius); ++y)
        for (int x = std::max(0, cx - radius); x <= std::min(color_z_m_.cols - 1, cx + radius); ++x) {
            const float z = color_z_m_.at<float>(y, x);
            if (z > 0) values.push_back(z);
        }
    if (static_cast<int>(values.size()) < options_.min_valid_pixels) return result;
    std::sort(values.begin(), values.end());
    const double z = values[values.size() / 2];
    const double spread = values.back() - values.front();
    if (spread > std::max(options_.max_spread_m, options_.max_spread_fraction * z)) return result;
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(std::vector<cv::Point2d>{{u, v}}, normalized, intrinsics(calibration_.color), calibration_.color.distortion);
    const auto ray = normalized.front();
    const double range = z * std::sqrt(1.0 + ray.x * ray.x + ray.y * ray.y);
    if (!std::isfinite(range) || range < options_.min_depth_m || range > options_.max_depth_m) return result;
    result.has_range = true;
    result.range_m = range;
    return result;
}

}  // namespace pure_uav
