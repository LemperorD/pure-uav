#include <cmath>
#include <cstdio>
#include <memory>

#include "feat/FeatureDatabase.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "update/UpdaterZeroVelocity.h"

namespace {
int failures = 0;

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

struct Fixture {
    static constexpr double gravity = 9.81;
    ov_msckf::StateOptions state_options;
    ov_msckf::UpdaterOptions updater_options;
    ov_msckf::NoiseManager noise;
    std::shared_ptr<ov_msckf::State> state = std::make_shared<ov_msckf::State>(state_options);
    std::shared_ptr<ov_core::FeatureDatabase> tracks = std::make_shared<ov_core::FeatureDatabase>();
    ov_msckf::UpdaterZeroVelocity updater{updater_options, noise, tracks, std::make_shared<ov_msckf::Propagator>(noise, gravity), gravity, 0.5, 10.0, 0.7, 2.0, 0.15, 0.03, 10, true};

    int sample = 0;
    float u = 0.0F;

    Fixture() {
        state->_timestamp = 1.0;
        auto imu = state->_imu->value();
        imu(7) = 0.7;
        state->_imu->set_value(imu);
        state->_imu->set_fej(imu);
        addTracks();
    }

    double time() const {
        return 1.0 + 0.005 * sample;
    }

    void addTracks() {
        for (size_t id = 0; id < 40; ++id) {
            const auto coordinate = static_cast<float>(id);
            tracks->update_feature(id, time(), 0, u + coordinate, coordinate, 0.0F, 0.0F);
        }
    }

    bool frame(double gyro_dps = 0.2, float displacement = 0.0F, int imu_steps = 7, bool cover_end = true) {
        const int end = sample + imu_steps;
        for (; sample < end + static_cast<int>(cover_end); ++sample) {
            ov_core::ImuData imu;
            imu.timestamp = time();
            imu.wm = Eigen::Vector3d(gyro_dps * std::acos(-1.0) / 180.0, 0.0, 0.0);
            imu.am = Eigen::Vector3d(0.0, 0.0, gravity);
            updater.feed_imu(imu);
        }
        sample = end;
        u += displacement;
        addTracks();
        const bool stationary = updater.try_update(state, time());
        // On a rejected ZUPT, normal VIO would advance the state to the frame time.
        state->_timestamp = time();
        return stationary;
    }
};

void checkNoisyStop() {
    Fixture f;
    f.updater.seed_still_streak();
    for (int i = 0; i < 9; ++i) {
        CHECK(!f.frame((i == 1 || i == 6) ? 2.5 : 0.2));
    }
    CHECK(f.frame());
    CHECK(f.state->_imu->vel().norm() < 0.01);
    CHECK(!f.frame(2.5));
    CHECK(f.frame());
}

void checkMotionAndDropouts() {
    Fixture f;
    for (int i = 0; i < 9; ++i) {
        CHECK(!f.frame());
    }
    CHECK(!f.frame(5.5));  // IMU veto even if visual tracks are static.
    for (int i = 0; i < 12; ++i) {
        CHECK(!f.frame(0.2, 1.0F));  // Motion visible in the image even with quiet IMU.
    }
    for (int i = 0; i < 9; ++i) {
        CHECK(!f.frame());
    }
    CHECK(!f.frame(0.2, 0.0F, 30));  // Camera gap.
    for (int i = 0; i < 9; ++i) {
        CHECK(!f.frame());
    }
    CHECK(!f.frame(0.2, 0.0F, 7, false));  // IMU ends before the image time.
    for (int i = 0; i < 9; ++i) {
        CHECK(!f.frame());
    }
    CHECK(f.frame());
    f.updater.reset_detection();
    CHECK(!f.frame());  // Evidence cannot survive a runtime off/on transition.
}

void checkMarginalFramesCannotTrigger() {
    Fixture f;
    for (int i = 0; i < 20; ++i) {
        CHECK(!f.frame(i % 3 == 0 ? 2.5 : 0.2));
    }
}
}  // namespace

int main() {
    checkNoisyStop();
    checkMotionAndDropouts();
    checkMarginalFramesCannotTrigger();
    return failures == 0 ? 0 : 1;
}
