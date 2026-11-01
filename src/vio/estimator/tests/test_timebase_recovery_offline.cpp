// 离线"飞控时间基变化"恢复测试（不需要飞控/相机，用已录数据集回放）。
//
// 目的：验证 VioEstimator::resetForNewTimebase() 真的能把估计器从旧时间基里拉出来、
// 按新时间基重新初始化并继续输出位姿。这是 vio_live 里"飞控重启/串口重连"恢复路径的
// 估计器部分——真机拔插还没条件做，但这一半可以离线验证。
//
// 做法：
//   1) 回放数据集直到估计器初始化并稳定若干帧；
//   2) 调 resetForNewTimebase()，并把后续所有 IMU/图像时间戳整体平移，
//      让新时间基从约 1.0 s 重新开始（等价于飞控 boot 时间从 ~0 重来）；
//   3) 继续回放，检查：立即不再发布 → 重新初始化 → 新时间基位姿 → 时间戳单调。
//
// 用法：
//   ./bin/test_timebase_recovery --dataset datasets/loop5 \
//        --config-dir config/gemini_1920x1080 [--warmup-frames 150] [--max-after 500]
//
// 没有数据集时返回 77（ctest 视为 skip）。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "offline_dataset.hpp"
#include "pure_uav/vio_estimator.hpp"

namespace {

int g_failed = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("  ❌ 第 %d 行不成立: %s\n", __LINE__, #cond);    \
            ++g_failed;                                                  \
        }                                                                \
    } while (0)

void usage(const char* argv0) {
    std::printf("用法: %s --dataset DIR --config-dir DIR [选项]\n"
                "  --warmup-frames N   初始化后先稳定多少帧再模拟重启（默认 150）\n"
                "  --max-after N       重启后最多回放多少帧（默认 600）\n"
                "  --quiet\n",
                argv0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string dataset_dir;
    std::string config_dir = "config/gemini_1920x1080";
    int warmup_frames = 150;
    int max_after = 600;
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s 需要一个参数\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--dataset") {
            dataset_dir = need("--dataset");
        } else if (a == "--config-dir") {
            config_dir = need("--config-dir");
        } else if (a == "--warmup-frames") {
            warmup_frames = std::stoi(need("--warmup-frames"));
        } else if (a == "--max-after") {
            max_after = std::stoi(need("--max-after"));
        } else if (a == "--quiet") {
            quiet = true;
        } else if (a == "-h" || a == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    if (dataset_dir.empty()) {
        std::printf("[skip] 没有给 --dataset（需要已录数据集才能跑）\n");
        return 77;
    }

    pure_uav::OfflineDataset dataset;
    std::string err;
    if (!dataset.load(dataset_dir, &err)) {
        std::fprintf(stderr, "读取数据集失败: %s\n", err.c_str());
        return 1;
    }
    const auto& images = dataset.images();
    const auto& imu = dataset.imu();
    std::printf("数据集: %s（图像 %zu 帧，IMU %zu 帧）\n", dataset.dir().c_str(), images.size(),
                imu.size());

    pure_uav::VioEstimator::Options opt;
    opt.config_dir = config_dir;
    opt.verbose = !quiet;
    opt.health.reinit_on_recover = false;  // 只测时间基重置这条独立路径
    pure_uav::VioEstimator est(opt);
    if (!est.ok()) {
        std::fprintf(stderr, "估计器初始化失败: %s\n", est.error().c_str());
        return 1;
    }

    size_t imu_idx = 0;
    bool reset_done = false;
    int64_t ts_offset_us = 0;
    int frames_since_init = 0;
    int rebuild_before = 0;
    int frames_after_reset = 0;
    int frames_after_reinit = 0;
    bool reinit_observed = false;
    bool monotonic_ok = true;
    uint64_t last_shifted_ts = 0;
    uint64_t first_new_base_ts = 0;
    int reset_frame = -1;

    for (size_t k = 0; k < images.size(); ++k) {
        const auto& img = images[k];

        if (!reset_done && est.initialized()) {
            ++frames_since_init;
            if (frames_since_init >= warmup_frames) {
                // ---- 模拟飞控重启：换估计器 + 后续时间戳整体回退到约 1.0 s ----
                rebuild_before = static_cast<int>(est.rebuildCount());
                reset_done = true;
                reset_frame = static_cast<int>(k);
                ts_offset_us = static_cast<int64_t>(img.timestamp_us) - 1000000;
                est.resetForNewTimebase("offline simulate FC reboot");
                CHECK(!est.initialized());
                CHECK(!est.publishable());
                std::printf("[reset] 第 %zu 帧：模拟飞控重启，后续时间戳平移 %lld us\n", k,
                            static_cast<long long>(ts_offset_us));
                // 这一帧起按新时间基喂
            }
        }

        // 喂 IMU（时间戳按需要平移）
        while (imu_idx < imu.size() && imu[imu_idx].timestamp_us <= img.timestamp_us) {
            pure_uav::ImuSample s = imu[imu_idx];
            if (reset_done) s.timestamp_us -= static_cast<uint64_t>(ts_offset_us);
            est.feedImu(s);
            ++imu_idx;
        }

        cv::Mat gray = cv::imread(img.path, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) {
            std::fprintf(stderr, "[WARN] 读图失败: %s\n", img.path.c_str());
            continue;
        }
        const uint64_t shifted_ts =
            reset_done ? img.timestamp_us - static_cast<uint64_t>(ts_offset_us) : img.timestamp_us;
        est.feedImage(gray, shifted_ts);
        if (reset_done) ++frames_after_reset;

        if (reset_done && est.initialized()) {
            if (!reinit_observed) {
                reinit_observed = true;
                first_new_base_ts = est.lastPoseTimestampUs();
                std::printf("[reinit] 重启后第 %d 帧重新初始化：首次位姿时间戳 %llu us"
                            "（旧时间基是 %.1f s 量级）\n",
                            frames_after_reset,
                            static_cast<unsigned long long>(first_new_base_ts),
                            static_cast<double>(img.timestamp_us) * 1e-6);
                CHECK(est.publishable());
            }
            ++frames_after_reinit;
            pure_uav::VisionPose p;
            if (est.getLatestPose(p)) {
                if (last_shifted_ts != 0 && p.timestamp_us <= last_shifted_ts) monotonic_ok = false;
                last_shifted_ts = p.timestamp_us;
            }
            if (frames_after_reinit >= max_after) break;
        }

        if (reset_done && !reinit_observed && frames_after_reset > 4000) {
            std::fprintf(stderr, "重启后 4000 帧仍未重新初始化，测试失败\n");
            break;
        }
    }

    // ---- 汇总与断言 ----
    std::printf("\n===== 结果 =====\n");
    std::printf("  模拟重启帧      : %d\n", reset_frame);
    std::printf("  重启后回放帧数  : %d\n", frames_after_reset);
    std::printf("  重新初始化      : %s（重启后 %d 帧）\n", reinit_observed ? "是" : "否",
                frames_after_reinit);
    std::printf("  rebuildCount    : %d → %d\n", rebuild_before,
                static_cast<int>(est.rebuildCount()));
    std::printf("  新时间基首帧    : %llu us\n",
                static_cast<unsigned long long>(first_new_base_ts));
    std::printf("  位姿时间戳单调  : %s\n", monotonic_ok ? "是" : "否");

    CHECK(reset_frame >= 0);
    CHECK(reinit_observed);
    CHECK(static_cast<int>(est.rebuildCount()) == rebuild_before + 1);
    CHECK(first_new_base_ts > 0);
    if (first_new_base_ts > 0) {
        // 时间戳应该落回新时间基（约 1 s 起），而不是继续旧时间基（~12353 s）
        CHECK(first_new_base_ts < 10000000ULL);
    }
    CHECK(monotonic_ok);

    if (g_failed == 0) {
        std::printf("PASS: 时间基重置后估计器按新时间基重新初始化并继续输出\n");
        return 0;
    }
    std::printf("FAILED: %d 处不通过\n", g_failed);
    return 1;
}
