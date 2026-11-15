#include <cmath>
#include <cstdio>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

#include "glue/camera_undistort.hpp"
#include "glue/target_adapter.hpp"

using namespace pure_uav::app;
using pure_uav::guidance::flight::GuidanceTargetSample;

namespace {
int checks = 0;

void expect(bool value, const char* reason) {
    ++checks;
    if (!value) {
        throw std::runtime_error(reason);
    }
}

CameraCalibration cameraCalibration() {
    CameraCalibration camera;
    camera.calibrated = true;
    camera.width = 640;
    camera.height = 480;
    camera.fx = 500.0;
    camera.fy = 500.0;
    camera.cx = 320.0;
    camera.cy = 240.0;
    camera.distortion[0] = 0.1;
    camera.distortion[1] = -0.05;
    camera.distortion[2] = 0.0;
    camera.distortion[3] = 0.0;
    camera.distortion[4] = 0.0;
    return camera;
}

MissionTargetPacket target(double u, double v) {
    MissionTargetPacket packet;
    packet.present = 1;
    packet.captured_ns = 1000;
    packet.sequence = 1;
    packet.u_px = u;
    packet.v_px = v;
    packet.w_px = 20.0;
    packet.h_px = 10.0;
    packet.confidence = 0.9;
    packet.class_id = 1;
    return packet;
}

cv::Point2d referenceUndistort(const CameraCalibration& camera, double u, double v) {
    const cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0);
    cv::Mat distortion(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        distortion.at<double>(0, i) = camera.distortion[i];
    }
    const std::vector<cv::Point2d> source{{u, v}};
    std::vector<cv::Point2d> corrected;
    cv::undistortPoints(source, corrected, intrinsics, distortion, cv::noArray(), intrinsics);
    return corrected.at(0);
}

}  // namespace

int main() {
    try {
        const CameraCalibration camera = cameraCalibration();
        TargetAdapter adapter(camera);

        GuidanceTargetSample sample;
        MissionTargetPacket packet = target(320.0, 240.0);
        expect(adapter.make(packet, 12345, sample), "center target should map");
        expect(sample.present && sample.has_box, "center target should be present");
        expect(sample.boot_us == 12345, "boot timestamp should be copied");
        expect(std::fabs(sample.u_px - 320.0) < 1e-9 && std::fabs(sample.v_px - 240.0) < 1e-9, "center pixel should not move");

        packet = target(600.0, 100.0);
        expect(adapter.make(packet, 12345, sample), "edge target should map");
        const cv::Point2d expected = referenceUndistort(camera, packet.u_px, packet.v_px);
        expect(std::fabs(sample.u_px - expected.x) < 1e-6 && std::fabs(sample.v_px - expected.y) < 1e-6, "edge pixel should be undistorted");

        packet = target(600.0, 100.0);
        packet.has_range = 1;
        packet.range_m = 2.5;
        expect(adapter.make(packet, 12345, sample), "target with range should map");
        expect(sample.has_range && std::fabs(sample.range_m - 2.5) < 1e-12, "range should be passed through when present");

        packet = target(600.0, 100.0);
        packet.present = 0;
        expect(adapter.make(packet, 12345, sample), "missing target should be a valid empty sample");
        expect(!sample.present, "missing target should not be present");

        packet = target(std::nan(""), 100.0);
        expect(!adapter.make(packet, 12345, sample), "non-finite pixel should be rejected");

        packet = target(700.0, 100.0);
        expect(!adapter.make(packet, 12345, sample), "out-of-image pixel should be rejected");

        CameraCalibration uncalibrated = camera;
        uncalibrated.calibrated = false;
        TargetAdapter raw_adapter(uncalibrated);
        packet = target(600.0, 100.0);
        expect(raw_adapter.make(packet, 12345, sample), "uncalibrated target should pass through raw pixels");
        expect(sample.present && sample.u_px == 600.0 && sample.v_px == 100.0, "uncalibrated pixels should stay raw");

        double u = 0.0;
        double v = 0.0;
        expect(undistortPixel(camera, 320.0, 240.0, u, v), "center undistort should succeed");
        expect(!undistortPixel(camera, 700.0, 100.0, u, v), "out-of-image undistort should fail");
        expect(!undistortPixel(uncalibrated, 600.0, 100.0, u, v), "uncalibrated undistort should fail");

        std::printf("PASS: %d target adapter/undistort checks\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
