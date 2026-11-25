#include "glue/camera_undistort.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

namespace pure_uav::app {
namespace {
bool finitePixel(const CameraCalibration& camera, double u, double v) {
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return false;
    }
    if (camera.width > 0 && (u < 0.0 || u >= static_cast<double>(camera.width))) {
        return false;
    }
    if (camera.height > 0 && (v < 0.0 || v >= static_cast<double>(camera.height))) {
        return false;
    }
    return true;
}
}  // namespace

bool undistortPixel(const CameraCalibration& camera, double u_px, double v_px, double& u_out, double& v_out) {
    if (!camera.calibrated || !(camera.fx > 0.0) || !(camera.fy > 0.0) || !finitePixel(camera, u_px, v_px)) {
        return false;
    }

    bool has_distortion = false;
    for (double coefficient : camera.distortion) {
        if (!std::isfinite(coefficient)) {
            return false;
        }
        has_distortion = has_distortion || coefficient != 0.0;
    }

    // 无畸变时不做无意义的点变换，避免 OpenCV 对平凡输入引入额外误差。
    if (!has_distortion) {
        u_out = u_px;
        v_out = v_px;
        return finitePixel(camera, u_out, v_out);
    }

    const cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0);
    cv::Mat distortion(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        distortion.at<double>(0, i) = camera.distortion[i];
    }

    const std::vector<cv::Point2d> source{{u_px, v_px}};
    std::vector<cv::Point2d> corrected;
    try {
        cv::undistortPoints(source, corrected, intrinsics, distortion, cv::noArray(), intrinsics);
    } catch (const cv::Exception&) {
        return false;
    }
    if (corrected.size() != 1 || !std::isfinite(corrected[0].x) || !std::isfinite(corrected[0].y)) {
        return false;
    }
    if (!finitePixel(camera, corrected[0].x, corrected[0].y)) {
        return false;
    }
    u_out = corrected[0].x;
    v_out = corrected[0].y;
    return true;
}

}  // namespace pure_uav::app
