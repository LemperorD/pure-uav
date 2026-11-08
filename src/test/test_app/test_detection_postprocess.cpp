#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include "yolo_postprocess.hpp"

using namespace pure_uav::detection;
namespace {
int checks = 0;
void check(bool condition, const char* reason) {
    ++checks;
    if (!condition) throw std::runtime_error(reason);
}
}  // namespace
int main() {
    try {
        std::vector<float> rows = {100, 60, 40, 20, 0.90f, 0.1f, 101, 60, 40, 20, 0.80f, 0.1f, 100, 60, 40, 20, 0.1f, 0.95f, 300, 50, 20, 10, 0.1f, 0.1f};
        NpuOutputView view;
        view.data = rows.data();
        view.channels = 6;
        view.anchors = 4;
        view.channel_major = false;
        view.letterbox_scale = 0.5f;
        PostprocessParams params;
        params.img_w = 640;
        params.img_h = 480;
        FrameResult result{};
        check(postprocess(view, params, result), "anchor-major decode");
        check(result.count == 2 && result.class_id == 1, "class-aware NMS or primary target");
        check(result.boxes[0].x1 == 160 && result.boxes[0].x2 == 240 && result.u_px == 200, "unletterbox coordinates");
        std::vector<float> channels(rows.size());
        for (int anchor = 0; anchor < 4; ++anchor)
            for (int channel = 0; channel < 6; ++channel) channels[channel * 4 + anchor] = rows[anchor * 6 + channel];
        view.data = channels.data();
        view.channel_major = true;
        check(postprocess(view, params, result) && result.count == 2, "channel-major decode");
        params.target_class = 0;
        check(postprocess(view, params, result) && result.count == 1 && result.class_id == 0, "target class filter");
        params.conf_thr = 0.99f;
        check(postprocess(view, params, result) && !result.present && result.class_id == -1, "no target must clear cached primary fields");
        params.conf_thr = 0.25f;
        channels[0] = std::numeric_limits<float>::quiet_NaN();
        check(!postprocess(view, params, result), "NaN tensor rejected");
        channels[0] = 100;
        view.letterbox_scale = 0;
        check(!postprocess(view, params, result), "zero letterbox scale rejected");
        view.letterbox_scale = 1;
        params.num_classes = 3;
        check(!postprocess(view, params, result), "model class-count mismatch rejected");
        params.num_classes = 2;
        params.max_det = 0;
        check(!postprocess(view, params, result), "zero max_det rejected");
        params.max_det = 100;
        params.conf_thr = std::numeric_limits<float>::infinity();
        check(!postprocess(view, params, result), "invalid threshold rejected");
        params.conf_thr = 0.25f;
        channels[4 * 4] = 1.5f;
        check(!postprocess(view, params, result), "nonprobability score rejected");
        std::printf("PASS: %d YOLO postprocess checks using known tensors; no inference claims\n", checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
