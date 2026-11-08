#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include "pure_uav/depth_range.hpp"
#ifdef PURE_VIO_TEST_APP_RANGE
#include "pure_uav/target_depth_range.hpp"
#endif

namespace {
void check(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
void near(double actual, double expected, double tolerance, const char* why) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr, "%s: %.8f expected %.8f\n", why, actual, expected);
        throw std::runtime_error(why);
    }
}
pure_uav::DepthRangeCalibration calibration() {
    pure_uav::DepthRangeCalibration c;
    c.color.width = c.depth.width = 41; c.color.height = c.depth.height = 31;
    c.color.fx = c.depth.fx = 10; c.color.fy = c.depth.fy = 10;
    c.color.cx = c.depth.cx = 20; c.color.cy = c.depth.cy = 15;
    return c;
}
void missing(const pure_uav::DepthRangeSample& s, const char* why) {
    check(!s.has_range && s.range_m == 0, why);
}
void testColorCalibrationFile() {
    using namespace pure_uav;
    struct Fixture {
        std::string path = cv::tempfile(".yaml");
        ~Fixture() { std::remove(path.c_str()); }
        void write(const std::string& text) { std::ofstream file(path); file << text; }
    } fixture;
    const std::string valid = "%YAML:1.0\n---\ncam0:\n"
        "  camera_model: pinhole\n  distortion_model: radtan\n"
        "  intrinsics: [1367.0, 1368.0, 971.0, 550.0]\n"
        "  distortion_coeffs: [0.09, -0.15, -0.001, 0.0008]\n"
        "  resolution: [1920, 1080]\n";
    RangeCameraModel model;
    std::string error = "old error";
    fixture.write(valid);
    check(loadRangeColorCalibration(fixture.path, 1920, 1080, model, &error), "valid Kalibr color calibration rejected");
    check(error.empty() && model.width == 1920 && model.height == 1080, "color calibration metadata");
    near(model.fx, 1367.0, 1e-10, "color focal length");
    near(model.distortion[2], -0.001, 1e-10, "color tangential distortion order");
    check(model.distortion[4] == 0, "radtan must not invent higher coefficients");
    check(!loadRangeColorCalibration(fixture.path, 640, 480, model, &error) && !model.valid() && !error.empty(),
          "resolution mismatch accepted or old calibration retained");
    for (const auto& replacement : {std::pair<std::string, std::string>{"radtan", "equidistant"},
                                  {"pinhole", "omni"}, {"1367.0", "-1.0"}, {"1367.0", ".Nan"},
                                  {"1367.0", "\"wrong\""}, {"[1920, 1080]", "[1920]"},
                                  {"[1367.0, 1368.0, 971.0, 550.0]", "[1367.0, 1368.0]"},
                                  {"cam0:", "cam1:"}}) {
        auto bad = valid;
        const auto pos = bad.find(replacement.first);
        check(pos != std::string::npos, "bad fixture setup");
        bad.replace(pos, replacement.first.size(), replacement.second);
        fixture.write(bad);
        model.fx = model.fy = 1; model.width = model.height = 1;
        check(!loadRangeColorCalibration(fixture.path, 1920, 1080, model, &error) && !model.valid(),
              "invalid color calibration accepted or previous output retained");
    }
    fixture.write("this is not yaml: [");
    check(!loadRangeColorCalibration(fixture.path, 1920, 1080, model), "malformed YAML accepted");
    check(!loadRangeColorCalibration(fixture.path + ".missing", 1920, 1080, model), "missing calibration accepted");
}

}

int main() {
    using namespace pure_uav;
    try {
        testColorCalibrationFile();
        cv::Mat raw(1, 3, CV_16UC1);
        raw.at<uint16_t>(0, 0) = 2000; raw.at<uint16_t>(0, 1) = 0; raw.at<uint16_t>(0, 2) = 65535;
        const auto mm = depthMillimeters(raw, 0.5);
        check(mm.at<uint16_t>(0, 0) == 1000 && mm.at<uint16_t>(0, 1) == 0 && mm.at<uint16_t>(0, 2) == 0,
              "SDK value scale or invalid-code normalization failed");
        check(depthMillimeters(raw, NAN).empty() && depthMillimeters(raw, 0).empty(), "invalid SDK units accepted");
        auto c = calibration();
        DepthRangeProjector projector;
        check(projector.configure(c), "valid calibration");
        cv::Mat plane(c.depth.height, c.depth.width, CV_16UC1, cv::Scalar(2000));
        check(projector.update(plane, 1000000), "plane update");
        auto center = projector.sample(20, 15, 1000000);
        check(center.has_range, "center has no range"); near(center.range_m, 2, 1e-6, "mm to m");
        const auto off_axis = projector.sample(30, 15, 1000000);
        check(off_axis.has_range, "off-axis has no range");
        near(off_axis.range_m, 2 * std::sqrt(2.0), 1e-6, "range is Euclidean, not axial Z");
        missing(projector.sample(-1, 15, 1000000), "out of image accepted");
        missing(projector.sample(NAN, 15, 1000000), "NaN pixel accepted");
        missing(projector.sample(20, 15, 0), "unknown target timestamp accepted");
        missing(projector.sample(20, 15, 1050001), "stale depth accepted");
        missing(projector.sample(20, 15, 949999), "future mismatched depth accepted");
        check(projector.sample(20, 15, 1050000).has_range, "timestamp tolerance boundary rejected");

#ifdef PURE_VIO_TEST_APP_RANGE
        app::TargetSample target;
        target.present = true; target.boot_us = 1000000; target.u_px = 30; target.v_px = 15;
        target.has_box = true; target.w_px = 12; target.conf = 0.8; target.class_id = 7;
        check(fillTargetDepthRange(target, projector), "app bridge rejected valid range");
        near(target.range_m, off_axis.range_m, 1e-6, "app range value");
        check(target.boot_us == 1000000 && target.u_px == 30 && target.has_box && target.w_px == 12 &&
              target.conf == 0.8 && target.class_id == 7, "bridge changed detection fields");
        target.boot_us = 2000000;
        check(!fillTargetDepthRange(target, projector) && !target.has_range && target.range_m == 0,
              "stale range leaked into new TargetSample");
        target.boot_us = 1000000; target.present = false; target.has_range = true; target.range_m = 9;
        check(!fillTargetDepthRange(target, projector) && !target.has_range && target.range_m == 0 && !target.present,
              "lost target retained range or became present");
#endif

        // 无效输入必须清空上一帧，不允许复用旧的目标距离。
        for (const cv::Mat& bad : std::vector<cv::Mat>{cv::Mat{}, cv::Mat::zeros(31, 41, CV_32FC1),
                                   cv::Mat::zeros(30, 41, CV_16UC1)}) {
            check(projector.update(plane, 1000000), "prepare prior frame");
            check(!projector.update(bad, 1000000), "invalid depth accepted");
            missing(projector.sample(20, 15, 1000000), "prior range retained after bad frame");
        }
        check(!projector.update(plane, 0), "unknown depth timestamp accepted");
        for (int value : {0, 65535, 100, 12000}) {
            cv::Mat invalid(31, 41, CV_16UC1, cv::Scalar(value));
            check(!projector.update(invalid, 1000000), "invalid/range-limited depth accepted");
            missing(projector.sample(20, 15, 1000000), "invalid pixel became range");
        }
        auto holes = plane.clone(); holes.at<uint16_t>(15, 20) = 0;
        check(projector.update(holes, 1000000), "small hole update");
        check(projector.sample(20, 15, 1000000).has_range, "small hole not handled by neighborhood");
        cv::Mat edge(31, 41, CV_16UC1, cv::Scalar(2000)); edge.colRange(21, 41).setTo(4000);
        check(projector.update(edge, 1000000), "edge update");
        missing(projector.sample(20, 15, 1000000), "background/foreground discontinuity accepted");

        // 两路不同分辨率与基线，必须投影而不是直接复制彩色像素下标。
        auto transformed = calibration();
        transformed.color.fx = 20; transformed.color.fy = 20;
        transformed.t_depth_to_color_m = {0.2, 0, 0.3};
        check(projector.configure(transformed), "translated calibration");
        check(projector.update(plane, 2000000), "translated plane");
        auto translated = projector.sample(25, 15, 2000000);
        check(translated.has_range, "translated range missing");
        near(translated.range_m, 2.3 * std::sqrt(1.0 + 0.25 * 0.25), 1e-5, "translation/scale projection");

        // 1080p 彩色约为深度焦距的 3 倍：7x7 在不利子像素相位下只有 4 个点。
        auto sparse = calibration();
        sparse.color.width = 123; sparse.color.height = 93;
        sparse.color.fx = sparse.color.fy = 30;
        sparse.color.cx = 61; sparse.color.cy = 46;
        DepthRangeProjector::Options old_patch; old_patch.patch_radius = 3;
        DepthRangeProjector too_small(old_patch);
        check(too_small.configure(sparse) && too_small.update(plane, 2500000), "sparse old patch setup");
        missing(too_small.sample(62, 47, 2500000), "fixture should expose sparse 7x7 support");
        check(projector.configure(sparse) && projector.update(plane, 2500000), "sparse 1080p setup");
        auto sparse_result = projector.sample(62, 47, 2500000);
        check(sparse_result.has_range, "default patch rejects valid low-resolution depth projected to 1080p");
        near(sparse_result.range_m, 2 * std::sqrt(1 + 2.0 / 900), 1e-5, "sparse projection range");
        cv::Mat few = cv::Mat::zeros(31, 41, CV_16UC1);
        few.at<uint16_t>(15, 20) = few.at<uint16_t>(15, 21) = 2000;
        few.at<uint16_t>(16, 20) = few.at<uint16_t>(16, 21) = 2000;
        check(projector.update(few, 2500000), "few points setup");
        missing(projector.sample(62, 47, 2500000), "enlarged patch must retain minimum valid support");

        // 单点旋转与遮挡测试采用无邻域查询，独立检查外参方向与 z-buffer。
        DepthRangeProjector::Options point_options; point_options.patch_radius = 0; point_options.min_valid_pixels = 1;
        DepthRangeProjector point(point_options);
        auto rotated = calibration(); rotated.R_depth_to_color = {0,-1,0, 1,0,0, 0,0,1};
        check(point.configure(rotated), "rotation calibration");
        cv::Mat single = cv::Mat::zeros(31, 41, CV_16UC1); single.at<uint16_t>(15, 22) = 2000;
        check(point.update(single, 3000000), "rotated point");
        auto rotated_point = point.sample(20, 17, 3000000);
        check(rotated_point.has_range, "rotation projected to wrong pixel");
        near(rotated_point.range_m, std::sqrt(4.16), 1e-6, "rotated range");
        auto occlusion = calibration();
        occlusion.depth.fx = occlusion.depth.fy = occlusion.color.fx = occlusion.color.fy = 100;
        occlusion.t_depth_to_color_m = {0.1,0,0};
        check(point.configure(occlusion), "occlusion calibration");
        single.setTo(0); single.at<uint16_t>(15, 23) = 2000; single.at<uint16_t>(15, 18) = 1000;
        check(point.update(single, 3000000), "occlusion points");
        auto front = point.sample(28, 15, 3000000);
        check(front.has_range, "occlusion target missing");
        near(front.range_m, std::sqrt(1 + 0.08 * 0.08), 1e-6, "far point overwrote foreground");

        // 彩色 Brown 畸变去投影：物理射线 (1,0,1)，k1=0.1 后像素 u=31。
        auto distorted = calibration(); distorted.color.distortion[0] = 0.1;
        check(point.configure(distorted), "distortion calibration");
        single.setTo(0); single.at<uint16_t>(15, 30) = 2000;
        check(point.update(single, 4000000), "distorted point");
        auto distorted_point = point.sample(31, 15, 4000000);
        check(distorted_point.has_range, "distortion projection ignored");
        near(distorted_point.range_m, 2 * std::sqrt(2.0), 1e-4, "distortion ray range");

        auto invalid_c = calibration(); invalid_c.R_depth_to_color = cv::Matx33d::zeros();
        check(!point.configure(invalid_c), "missing extrinsic rotation accepted");
        missing(point.sample(31, 15, 4000000), "failed calibration retained prior output");
        check(!point.configure(DepthRangeCalibration{}), "unknown camera model accepted");
        check(depthTimestampFromColor(1000000, 3000000, 2990000) == 990000, "depth clock conversion older");
        check(depthTimestampFromColor(1000000, 3000000, 3010000) == 1010000, "depth clock conversion newer");
        check(depthTimestampFromColor(1000000, 0, 1) == 0, "unknown device time substituted");
        check(depthTimestampFromColor(10, 100, 50) == 0, "timestamp underflow");
        check(depthTimestampFromColor(UINT64_MAX, 100, 101) == 0, "timestamp overflow");
        std::puts("PASS: depth projection, Euclidean range, timestamps, invalidation and TargetSample bridge");
        return 0;
    } catch (const std::exception& e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
