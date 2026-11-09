#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "target_bridge.hpp"

using namespace pure_uav;
namespace {
void check(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
}  // namespace
int main() {
    try {
        detection::FrameResult result{};
        result.present = true;
        result.count = 1;
        result.u_px = 20;
        result.v_px = 30;
        result.w_px = 10;
        result.h_px = 12;
        result.conf = 0.9;
        result.class_id = 1;
        guidance::flight::GuidanceTargetSample output;
        check(detection::to_target_sample(result, 1000, output) && output.present && output.has_box && output.boot_us == 1000 && !output.has_range, "valid target bridge");
        for (int scenario = 0; scenario < 10; ++scenario) {
            auto invalid = result;
            if (scenario == 0) invalid.u_px = NAN;
            if (scenario == 1) invalid.v_px = INFINITY;
            if (scenario == 2) invalid.w_px = -1;
            if (scenario == 3) invalid.h_px = 0;
            if (scenario == 4) invalid.conf = NAN;
            if (scenario == 5) invalid.conf = 1.01;
            if (scenario == 6) invalid.conf = -0.01;
            if (scenario == 7) invalid.class_id = -1;
            if (scenario == 8) invalid.count = detection::kMaxDetections + 1;
            if (scenario == 9) invalid.present = false;
            output.present = output.has_box = true;
            check(!detection::to_target_sample(invalid, 2000, output) && !output.present && !output.has_box && output.class_id == -1 && output.conf == 0 && output.boot_us == 2000, "invalid input retained stale target");
        }
        std::puts("PASS: target bridge validates finite pixels, positive sizes, confidence, class and count");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
