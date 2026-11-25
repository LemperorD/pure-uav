// Regression: a nominal 60-Hz stream can have sub-16.67-ms intervals. Its
// production configuration must not silently throw away adjacent frames.
#include <cstdio>
#include <opencv2/core.hpp>

#include "pure_uav/vio_estimator.hpp"

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    pure_uav::VioEstimator::Options options;
    options.config_dir = argv[1];
    options.verbose = false;
    options.init_threads = 1;
    pure_uav::VioEstimator estimator(options);
    if (!estimator.ok()) {
        std::fprintf(stderr, "FAIL: %s\n", estimator.error().c_str());
        return 1;
    }
    cv::Mat image(480, 640, CV_8UC1);
    cv::RNG random(42);
    random.fill(image, cv::RNG::UNIFORM, 0, 256);
    uint64_t timestamp = 1000000;
    const uint64_t intervals[] = {16000, 16000, 20000, 12000};
    for (int i = 0; i < 120; ++i) {
        if (!estimator.feedImage(image, timestamp)) return 1;
        timestamp += intervals[i % 4];
    }
    if (estimator.imageCount() != 120) {
        std::fprintf(stderr, "FAIL: accepted %d/120 camera frames\n", estimator.imageCount());
        return 1;
    }
    std::puts("PASS: all 120 nominal-60-Hz camera frames accepted");
    return 0;
}
