// 重建新 OpenVINS 实例后，运行期开关必须保留，不能退回 try_zupt 的配置默认值。
// 不需要飞控、相机或数据集；参数是含 try_zupt: true 的测试配置目录。
#include <cstdio>

#include <opencv2/imgcodecs.hpp>
#include "offline_dataset.hpp"

#include "pure_uav/vio_estimator.hpp"

namespace {
int failures = 0;
#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
            ++failures; \
        } \
    } while (0)

// 可选真实数据回放：抬高本测试的特征门限，走实际 LOST 重建路径。
void checkLostRebuild(pure_uav::VioEstimator::Options options, const char* dataset_dir) {
    pure_uav::OfflineDataset dataset;
    std::string error;
    if (!dataset.load(dataset_dir, &error)) {
        std::fprintf(stderr, "dataset failed: %s\n", error.c_str());
        ++failures;
        return;
    }
    options.health.min_features = 1000000;
    options.health.lost_after_s = 0.05;
    options.health.force_reinit_after_s = 0.05;
    options.health.reinit_on_recover = true;
    options.health.inflate_enable = false;
    pure_uav::VioEstimator estimator(options);
    if (!estimator.ok()) {
        std::fprintf(stderr, "estimator failed: %s\n", estimator.error().c_str());
        ++failures;
        return;
    }
    estimator.setZuptEnabled(false);
    size_t imu_index = 0;
    bool initialized = false;
    for (const auto& image : dataset.images()) {
        while (imu_index < dataset.imu().size() &&
               dataset.imu()[imu_index].timestamp_us <= image.timestamp_us) {
            estimator.feedImu(dataset.imu()[imu_index++]);
        }
        const cv::Mat gray = cv::imread(image.path, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) { ++failures; break; }
        estimator.feedImage(gray, image.timestamp_us);
        initialized = initialized || estimator.initialized();
        if (estimator.rebuildCount() > 0) break;
    }
    CHECK(initialized);
    CHECK(estimator.rebuildCount() == 1);
    CHECK(!estimator.zuptEnabled());
    CHECK(!estimator.publishable());
    std::printf("LOST rebuild: initialized=%d rebuilds=%lld ZUPT=%d\n",
                initialized, static_cast<long long>(estimator.rebuildCount()),
                estimator.zuptEnabled());
}
}

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::fprintf(stderr, "usage: %s CONFIG_DIR [DATASET_DIR]\n", argv[0]);
        return 2;
    }
    pure_uav::VioEstimator::Options options;
    options.config_dir = argv[1];
    options.verbose = false;
    options.init_threads = 1;
    pure_uav::VioEstimator estimator(options);
    if (!estimator.ok()) {
        std::fprintf(stderr, "estimator failed: %s\n", estimator.error().c_str());
        return 1;
    }
    CHECK(estimator.zuptEnabled());

    // 已解锁时禁用，模拟重连后仍然已解锁（上层不会再次切换开关）。
    estimator.setZuptEnabled(false);
    CHECK(!estimator.zuptEnabled());
    const auto rebuilds = estimator.rebuildCount();
    estimator.resetForNewTimebase("test: reconnect while armed");
    CHECK(!estimator.zuptEnabled());
    CHECK(!estimator.publishable());
    CHECK(estimator.rebuildCount() == rebuilds + 1);
    estimator.resetForNewTimebase("test: repeated reset while armed");
    CHECK(!estimator.zuptEnabled());

    // 地面重新允许零速更新后，下一次重建也应保留开启状态。
    estimator.setZuptEnabled(true);
    estimator.resetForNewTimebase("test: reset while disarmed");
    CHECK(estimator.zuptEnabled());
    estimator.setZuptEnabled(false);
    estimator.resetForNewTimebase("test: armed again");
    CHECK(!estimator.zuptEnabled());
    CHECK(estimator.rebuildCount() == rebuilds + 4);
    if (argc == 3) checkLostRebuild(options, argv[2]);
    if (failures) {
        std::fprintf(stderr, "FAILED: %d checks\n", failures);
        return 1;
    }
    std::printf("PASS: ZUPT runtime state survives estimator rebuilds (off/on/off)\n");
    return 0;
}
