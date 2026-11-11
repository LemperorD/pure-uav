#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

#include "glue/guidance_session.hpp"
#include "runtime/guidance_camera.hpp"
#include "startup/guidance_application.hpp"

namespace {
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
struct Fixture {
    std::string path = "/tmp/pure_uav_guidance_camera_" + std::to_string(getpid()) + ".yaml";
    ~Fixture() {
        std::remove(path.c_str());
    }
    void write(bool calibrated) const {
        std::ofstream file(path);
        file << "calibrated: " << (calibrated ? "true" : "false")
             << "\n"
                "source: synthetic\nwidth: 1920\nheight: 1080\ncamera_model: pinhole\ndistortion_model: radtan\n"
                "fx: 1367\nfy: 1368\ncx: 971\ncy: 550\n"
                "distortion: [0.09, -0.15, -0.001, 0.0008, 0.002]\nR_body_cam: [1,0,0,0,1,0,0,0,1]\n";
    }
};
}  // namespace
int main() {
    try {
        Fixture fixture;
        fixture.write(true);
        pure_uav::app::GuidanceSession session;
        std::string error;
        check(session.initCamera(fixture.path, error) == pure_uav::app::CameraLoadResult::kOk, error.c_str());
        const auto cam = session.params().loop.cam;
        check(cam.width == 1920 && cam.height == 1080 && cam.distortion[0] == 0.09 && cam.distortion[4] == 0.002, "loaded distortion/dimensions discarded");
        pure_uav::camera::StreamMeta stream;
        stream.range_calibration_valid = 1;
        stream.width = cam.width;
        stream.height = cam.height;
        stream.color_fx = cam.fx;
        stream.color_fy = cam.fy;
        stream.color_cx = cam.cx;
        stream.color_cy = cam.cy;
        std::copy(std::begin(cam.distortion), std::end(cam.distortion), stream.color_distortion);
        check(pure_uav::app::guidanceCameraMatchesStream(cam, stream, error), "matching color calibration rejected");
        stream.color_distortion[1] += 0.01;
        check(!pure_uav::app::guidanceCameraMatchesStream(cam, stream, error) && !error.empty(), "different measurement/guidance calibrations accepted");
        stream.color_distortion[1] = cam.distortion[1];
        stream.width = 640;
        check(!pure_uav::app::guidanceCameraMatchesStream(cam, stream, error), "different stream dimensions accepted");
        const cv::Matx33d K(cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
        // 对已知光学射线正向加畸变，检查制导恢复射线。覆盖中心/边缘及切向畸变。
        const std::vector<cv::Point3d> points{{0, 0, 1}, {0.5, 0.2, 1}, {-0.6, -0.3, 1}, {0.6, -0.35, 1}, {-0.6, 0.35, 1}};
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(points, cv::Vec3d(), cv::Vec3d(), K, cv::Mat(1, 5, CV_64F, const_cast<double*>(cam.distortion)), pixels);
        for (size_t i = 0; i < points.size(); ++i) {
            pure_uav::Vec3 ray;
            check(cam.ray(pixels[i].x, pixels[i].y, ray), "valid ray rejected");
            check(std::abs(ray.x - points[i].x) < 1e-8 && std::abs(ray.y - points[i].y) < 1e-8 && ray.z == 1, "guidance did not invert radtan distortion");
        }
        pure_uav::Vec3 ray;
        check(!cam.ray(-1, 0, ray) && !cam.ray(1920, 0, ray) && !cam.ray(NAN, 0, ray), "bad target pixel accepted");
        auto bad = cam;
        bad.distortion[0] = NAN;
        check(!bad.ray(971, 550, ray), "invalid distortion accepted");
        fixture.write(false);
        check(session.initCamera(fixture.path, error) == pure_uav::app::CameraLoadResult::kNotCalibrated && !session.params().loop.cam.valid(), "uninstalled camera became guidance-ready");
        std::puts("PASS: loaded guidance calibration, radtan rays, pixel bounds and uncalibrated gating");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
