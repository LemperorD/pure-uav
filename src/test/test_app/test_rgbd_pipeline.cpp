#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "camera_geometry.hpp"
#include "frame_publisher.hpp"
#include "pure_uav/target_depth_range.hpp"
#include "shm_color_frame.hpp"
#include "target_bridge.hpp"

namespace {
using namespace pure_uav;
using namespace pure_uav::camera;
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
void near(double actual, double expected, const char* message) {
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-6, message);
}
void testPipeline() {
    PublisherConfig cfg;
    const auto suffix = std::to_string(getpid());
    cfg.vio_name = "/pure_uav_test_rgbd_vio_" + suffix;
    cfg.color_name = "/pure_uav_test_rgbd_color_" + suffix;
    cfg.vio_payload_bytes = cfg.color_payload_bytes = 4096;
    FramePublisher publisher;
    std::string error;
    check(publisher.create(cfg, &error), error.c_str());
    StreamMeta stream;
    stream.width = stream.height = stream.depth_width = stream.depth_height = 16;
    stream.fps = 30;
    stream.has_depth = stream.range_calibration_valid = 1;
    stream.fx = stream.fy = stream.color_fx = stream.color_fy = 10;
    stream.cx = stream.color_cx = 5;
    stream.cy = stream.color_cy = 6;
    publisher.activate(stream);
    ShmChannelReader color, vio;
    check(color.open(cfg.color_name, ChannelType::color, 100, &error), error.c_str());
    check(vio.open(cfg.vio_name, ChannelType::vio, 100, &error), error.c_str());
    OrbbecCamera::CameraFrames f;
    f.gray = cv::Mat(16, 16, CV_8UC1, cv::Scalar(42));
    f.depth_mm = cv::Mat(16, 16, CV_16UC1, cv::Scalar(2000));
    f.color_encoded = cv::Mat(1, 16 * 16 * 3, CV_8UC1, cv::Scalar(77));
    f.encoding = ColorEncoding::bgr;
    f.width = f.height = 16;
    f.seq = 1;
    f.host_ns = monotonicNs();
    f.device_us = f.depth_device_us = 1000000;
    f.has_depth = true;
    publisher.publish(f);
    FrameMeta meta;
    uint64_t session = 0, generation = 0;
    std::vector<uint8_t> payload;
    check(vio.readLatest(meta, payload, session, generation), "VIO frame missing");
    check(meta.gray_bytes == 256 && meta.depth_bytes == 512 && meta.color_bytes == 0 && payload.front() == 42, "VIO payload changed");
    check(color.readLatest(meta, payload, session, generation), "RGB-D frame missing");
    check(meta.seq == f.seq && meta.host_ns == f.host_ns && meta.depth_device_us == f.depth_device_us, "RGB-D identity changed");
    EncodedColorFrame encoded;
    check(detection::unpackColorFrame(color.streamMeta(), meta, payload, encoded, error), error.c_str());
    check(encoded.bytes.at<uint8_t>(0) == 77 && encoded.depth_mm.at<uint16_t>(0, 0) == 2000, "color offset or depth bytes wrong");
    DepthRangeProjector projector;
    check(projector.configure(rangeCalibration(color.streamMeta())), "IPC calibration missing");
    const auto color_us = static_cast<uint64_t>(f.host_ns / 1000);
    check(projector.update(encoded.depth_mm, depthTimestampFromColor(color_us, encoded.device_us, encoded.depth_device_us)), "projector update failed");
    detection::FrameResult result{};
    result.present = true;
    result.count = 1;
    result.seq = f.seq;
    result.u_px = 5;
    result.v_px = 6;
    result.w_px = result.h_px = 6;
    result.conf = 0.9;
    result.class_id = 1;
    check(fillTargetDepthRange(result, projector, color_us), "range missing from detection");
    near(result.range_m, 2, "meters or straight-line range wrong");
    app::MissionTargetChannel target_channel;
    check(target_channel.open(), "target channel open failed");
    check(app::publishMissionTarget(target_channel.publisherFd(), detection::to_mission_target_packet(result, f.host_ns)), "target publish failed");
    app::MissionTargetPacket received;
    check(target_channel.latest(received, monotonicNs(), 1000000000), "target receive failed");
    check(received.present && received.has_range && received.sequence == f.seq && received.captured_ns == f.host_ns, "target packet lost frame identity/range");
    near(received.range_m, 2, "target packet range wrong");
    guidance::flight::GuidanceTargetSample target;
    check(detection::to_target_sample(result, 12345, target) && target.has_range, "guidance bridge lost range");
    near(target.range_m, 2, "guidance range wrong");

    // 外参方向和毫米→米只在边界换一次。
    auto transformed = stream;
    const double R[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    std::copy(std::begin(R), std::end(R), transformed.R_c2d);
    transformed.t_mm[0] = 100;
    transformed.t_mm[1] = 200;
    const auto c = rangeCalibration(transformed);
    check(c.valid(), "nonidentity extrinsic rejected");
    near(c.R_depth_to_color(0, 1), 1, "C2D rotation not inverted");
    near(c.t_depth_to_color_m[0], -0.2, "C2D translation not inverted");
    near(c.t_depth_to_color_m[1], 0.1, "C2D translation units wrong");
    transformed.range_calibration_valid = 0;
    check(!rangeCalibration(transformed).valid(), "missing SDK calibration fabricated");
    transformed = stream;
    std::fill(std::begin(transformed.R_c2d), std::end(transformed.R_c2d), 0);
    check(!rangeCalibration(transformed).valid(), "invalid rotation accepted");

    auto malformed = meta;
    malformed.depth_bytes -= 2;
    check(!detection::unpackColorFrame(stream, malformed, payload, encoded, error) && encoded.bytes.empty(), "bad depth length accepted");
    malformed = meta;
    malformed.flags = 0;
    check(!detection::unpackColorFrame(stream, malformed, payload, encoded, error), "missing depth flag accepted");
    malformed = meta;
    malformed.gray_bytes = 1;
    check(!detection::unpackColorFrame(stream, malformed, payload, encoded, error), "unexpected gray segment accepted");
    malformed = meta;
    malformed.color_encoding = 99;
    check(!detection::unpackColorFrame(stream, malformed, payload, encoded, error), "unknown color encoding accepted");

    // 独立限频只影响检测通道；无深度帧必须清除上次测距。
    f.seq = 2;
    f.host_ns += 1;
    publisher.publish(f);
    check(!color.readLatest(meta, payload, session, generation), "color rate limiter failed");
    check(vio.readLatest(meta, payload, session, generation) && meta.seq == 2, "color limiter stalled VIO");
    f.seq = 3;
    f.host_ns += 100000000;
    f.depth_mm.release();
    f.has_depth = false;
    f.depth_device_us = 0;
    publisher.publish(f);
    check(color.readLatest(meta, payload, session, generation), "color-only frame missing");
    check(detection::unpackColorFrame(stream, meta, payload, encoded, error) && encoded.depth_mm.empty() && encoded.bytes.at<uint8_t>(0) == 77, "color-only offset wrong");
    check(!projector.update(encoded.depth_mm, 0), "empty depth retained projection");
    check(!fillTargetDepthRange(result, projector, color_us) && !result.has_range && result.range_m == 0, "lost depth retained distance");
    check(!detection::to_mission_target_packet(result, f.host_ns).has_range, "missing range fabricated in mission packet");
}
}  // namespace
int main() {
    try {
        testPipeline();
        std::puts("PASS: RGB-D publisher/IPC/offsets/calibration/range/mission/guidance pipeline");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
