#include "pure_uav/vio_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <exception>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <thread>

#include "fcu_clock.hpp"
#include "live_timing.hpp"

namespace pure_uav {
namespace {
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool finiteImu(const ImuSample& s) {
    return s.timestamp_us > 0 && std::isfinite(s.gx) && std::isfinite(s.gy) && std::isfinite(s.gz) && std::isfinite(s.ax) && std::isfinite(s.ay) && std::isfinite(s.az);
}
}  // namespace

bool VioDepthInfo::valid() const {
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0 || fy <= 0 || width <= 0 || height <= 0) return false;
    Eigen::Matrix3d rotation;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) rotation(r, c) = R_c2d[3 * r + c];
    for (double t : t_c2d_m)
        if (!std::isfinite(t)) return false;
    return rotation.allFinite() && (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 0.02 && std::abs(rotation.determinant() - 1.0) < 0.02;
}

struct VioRuntime::Impl {
    VioEstimator::Options options;
    VioPublicationPolicy policy;
    VioImuSource& imu;
    VioFrameSource& frames;
    std::thread worker;
    std::atomic<bool> stop{false}, running{false}, publishable{false}, clock_valid{false};
    std::atomic<uint64_t> generation{0};
    mutable std::mutex publication_mutex;
    VioPoseSnapshot snapshot;
    FcuClock clock;
    mutable std::mutex stats_mutex;
    Stats cached_stats;
    std::mutex startup_mutex;
    std::condition_variable startup_cv;
    bool startup_done = false;
    std::string startup_error;
    std::mutex idle_mutex;
    std::condition_variable idle_cv;

    Impl(const VioEstimator::Options& o, VioImuSource& i, VioFrameSource& f, const VioPublicationPolicy& p) : options(o), policy(p), imu(i), frames(f) {
    }
    void invalidate() {
        std::lock_guard<std::mutex> lock(publication_mutex);
        clock_valid.store(false);
        publishable.store(false);
        generation.fetch_add(1);
        snapshot.valid = false;
        snapshot.health = static_cast<int>(VioHealthState::UNKNOWN);
        snapshot.features = 0;
    }
    void run() noexcept;
};

void VioRuntime::Impl::run() noexcept {
    Stats st;
    const auto saveStats = [&] {
        std::lock_guard<std::mutex> lock(stats_mutex);
        cached_stats = st;
    };
    const auto startup = [&](const std::string& error) {
        std::lock_guard<std::mutex> lock(startup_mutex);
        if (startup_done) return;
        startup_error = error;
        startup_done = true;
        startup_cv.notify_one();
    };
    try {
        if (!std::filesystem::is_regular_file(std::filesystem::path(options.config_dir) / options.estimator_config)) throw std::runtime_error("VIO estimator configuration file does not exist");
        VioEstimator estimator(options);
        if (!estimator.ok()) throw std::runtime_error(estimator.error());
        clock.reset();
        ImuTimestampFilter filter;
        std::optional<VioFrame> pending;
        uint64_t mapped_us = 0, last_raw_us = 0, last_fed_us = 0, first_fed_us = 0, last_pose_us = 0;
        uint64_t session = imu.session();
        uint64_t source_drops = imu.dropped();
        int64_t cutoff_ns = nowNs(), pending_since_ns = 0;
        int warmup = 150;
        bool recovering = true;
        int zupt_state = -1;
        std::deque<uint64_t> rate_window;
        frames.discardBefore(cutoff_ns);
        startup({});  // all consumer state is ready before start() returns

        const auto reset = [&](const char* why, int64_t cutoff) {
            // This lock is also held by main across the final gate and serial write.
            invalidate();
            recovering = true;
            ++st.timebase_resets;
            cutoff_ns = std::max(cutoff_ns, cutoff);
            clock.reset();
            filter.reset();
            pending.reset();
            frames.discardBefore(cutoff_ns);
            mapped_us = last_raw_us = last_fed_us = first_fed_us = last_pose_us = 0;
            rate_window.clear();
            st.imu_hz = 0;
            warmup = 150;
            estimator.resetForNewTimebase(why);
            if (!estimator.ok()) throw std::runtime_error(estimator.error());
        };
        const auto updateRate = [&](uint64_t timestamp) {
            if (!rate_window.empty() && timestamp <= rate_window.back()) rate_window.clear();
            rate_window.push_back(timestamp);
            while (rate_window.size() > 2 && timestamp - rate_window.front() > 1000000) rate_window.pop_front();
            if (rate_window.size() >= 8 && timestamp - rate_window.front() >= 20000) {
                st.imu_hz = static_cast<double>(rate_window.size() - 1) * 1e6 / static_cast<double>(timestamp - rate_window.front());
            }
        };
        while (!stop.load()) {
            bool did_work = false;
            const auto current_session = imu.session();
            if (current_session != session) {
                session = current_session;
                reset("connection session changed", nowNs());
            }
            const uint64_t drops = imu.dropped();
            if (drops != source_drops) {
                if (drops > source_drops) st.imu_dropped += drops - source_drops;
                source_drops = drops;
                reset("IMU source queue overflow", nowNs());
            }
            const int want_zupt = imu.armed() ? 0 : 1;
            if (want_zupt != zupt_state) {
                estimator.setZuptEnabled(want_zupt == 1);
                zupt_state = want_zupt;
                st.zupt_enabled = want_zupt == 1;
                ++st.zupt_changes;
            }
            VioImuSource::TimedImu item;
            while (!stop.load() && imu.pop(item)) {
                did_work = true;
                ++st.imu_popped;
                if (item.session != imu.session()) {
                    ++st.imu_dropped;
                    continue;
                }
                if (item.session != session) {
                    session = item.session;
                    reset("IMU connection session changed", item.host_ns);
                }
                if (item.host_ns < cutoff_ns) {
                    ++st.imu_dropped;
                    continue;
                }
                if (!finiteImu(item.imu) || item.host_ns <= 0 || item.host_ns > nowNs()) {
                    ++st.imu_dropped;
                    reset("invalid IMU sample", nowNs());
                    continue;
                }
                if (warmup > 0) {
                    updateRate(item.imu.timestamp_us);
                    --warmup;
                    ++st.imu_dropped;
                    continue;
                }
                const bool gap = last_raw_us != 0 && item.imu.timestamp_us > last_raw_us && item.imu.timestamp_us - last_raw_us > 100000;
                const auto result = clock.addSample(item.host_ns, item.imu.timestamp_us);
                if (gap || result == FcuClock::AddResult::kTimebaseReset) {
                    reset(gap ? "IMU gap exceeds 100ms" : "FCU clock jump", item.host_ns);
                    continue;  // rebuild, then warm up the new stream before feeding anything
                }
                if (result == FcuClock::AddResult::kOutOfOrder) {
                    ++st.imu_out_of_order;
                    continue;
                }
                last_raw_us = item.imu.timestamp_us;
                updateRate(item.imu.timestamp_us);
                ImuSample sample = item.imu;
                // Never reconstruct the grid from the requested rate. Wait until
                // the source timestamps provide an actual rate measurement.
                if (!(st.imu_hz > 20 && st.imu_hz <= 2000)) {
                    ++st.imu_dropped;
                    continue;
                }
                if (!filter.filter(sample.timestamp_us, 1e6 / st.imu_hz, sample.timestamp_us)) {
                    ++st.imu_out_of_order;
                    continue;
                }
                // Coverage stays in the original FCU domain; estimator inputs use Unix microseconds.
                const uint64_t fcu_us = sample.timestamp_us;
                if (!clock.fcuToUnixUs(fcu_us, sample.timestamp_us)) {
                    ++st.imu_dropped;
                    continue;
                }
                estimator.feedImu(sample);
                last_fed_us = fcu_us;
                if (first_fed_us == 0) first_fed_us = fcu_us;
            }
            if (imu.session() != session) continue;
            if (!pending) {
                VioFrame frame;
                if (frames.pop(frame, 1)) {
                    did_work = true;
                    if (frame.host_ns < cutoff_ns) {
                        ++st.frames_dropped;
                    } else {
                        pending = std::move(frame);
                        pending_since_ns = nowNs();
                        mapped_us = 0;
                    }
                }
            }
            if (pending) {
                const int64_t waited = nowNs() - pending_since_ns;
                if (pending->gray.empty() || pending->gray.type() != CV_8UC1 || pending->host_ns <= 0 || pending->host_ns > nowNs()) {
                    throw std::runtime_error("Invalid camera frame or host timestamp");
                }
                uint64_t image_unix_us = 0;
                if (!clock.valid() || (mapped_us == 0 && !clock.hostToFcuUs(pending->host_ns, mapped_us)) || !clock.fcuToUnixUs(mapped_us, image_unix_us)) {
                    if (waited > 2000000000LL) {
                        pending.reset();
                        ++st.frames_dropped;
                    }
                } else if (first_fed_us != 0 && static_cast<double>(mapped_us) + estimator.cameraTimeOffsetSec() * 1e6 < static_cast<double>(first_fed_us)) {
                    // Frames captured during stream warmup have no IMU history.
                    pending.reset();
                    ++st.frames_dropped;
                } else if (!imuCoversImage(last_fed_us, mapped_us, estimator.cameraTimeOffsetSec())) {
                    if (waited > 100000000LL) {
                        pending.reset();
                        ++st.frames_dropped;
                    }
                } else {
                    const auto depth = frames.depthInfo();
                    if (options.use_depth_init && !depth.valid()) {
                        if (waited > 100000000LL) {
                            pending.reset();
                            ++st.frames_dropped;
                        }
                    } else {
                        if (options.use_depth_init && !pending->depth_mm.empty()) {
                            if (pending->depth_mm.type() != CV_16UC1 || pending->depth_mm.cols != depth.width || pending->depth_mm.rows != depth.height) throw std::runtime_error("Depth frame does not match native depth calibration");
                            // Both device stamps must be from the same hardware clock.
                            if (pending->device_us > 0 && pending->depth_device_us > 0) {
                                const int64_t delta = pending->depth_device_us - pending->device_us;
                                uint64_t depth_unix_us = 0;
                                if (std::abs(delta) < 200000 && static_cast<int64_t>(mapped_us) + delta > 0 && clock.fcuToUnixUs(static_cast<uint64_t>(static_cast<int64_t>(mapped_us) + delta), depth_unix_us)) {
                                    estimator.setDepthImage(pending->depth_mm, depth_unix_us, depth.fx, depth.fy, depth.cx, depth.cy, depth.R_c2d, depth.t_c2d_m);
                                    ++st.depth_frames_used;
                                } else
                                    ++st.depth_frames_rejected;
                            } else
                                ++st.depth_frames_rejected;
                        } else if (options.use_depth_init)
                            ++st.depth_frames_rejected;
                        const int64_t image_host_ns = pending->host_ns;
                        const bool fed = estimator.feedImage(pending->gray, image_unix_us);
                        pending.reset();
                        if (fed)
                            ++st.frames_fed;
                        else
                            ++st.frames_dropped;
                        if (!estimator.ok()) throw std::runtime_error(estimator.error());
                        if (clock.valid() && estimator.initialized()) recovering = false;
                        VisionPose pose;
                        const auto health = estimator.healthReport();
                        const bool can_publish = !recovering && clock.valid() && estimator.publishable();
                        std::lock_guard<std::mutex> lock(publication_mutex);
                        snapshot.health = static_cast<int>(health.state);
                        snapshot.features = health.features;
                        if (fed && estimator.initialized() && estimator.getLatestPose(pose) && pose.timestamp_us != last_pose_us && imu.session() == session) {
                            last_pose_us = pose.timestamp_us;
                            snapshot.pose = pose;
                            snapshot.rebuilds = estimator.rebuildCount();
                            snapshot.session = session;
                            snapshot.timebase_generation = generation.load();
                            snapshot.sample_host_ns = image_host_ns;
                            snapshot.produced_host_ns = nowNs();
                            snapshot.valid = can_publish;
                            const bool ok = health.state == VioHealthState::OK;
                            snapshot.quality = ok ? policy.quality_ok : policy.quality_degraded;
                            snapshot.pos_sigma_m = ok ? policy.pos_sigma_ok_m : policy.pos_sigma_degraded_m;
                            snapshot.vel_sigma_mps = ok ? policy.vel_sigma_ok_mps : policy.vel_sigma_degraded_mps;
                            ++st.poses;
                            if (!can_publish) ++st.blocked;
                        }
                        if (!can_publish || snapshot.rebuilds != estimator.rebuildCount()) snapshot.valid = false;
                        clock_valid.store(clock.valid());
                        publishable.store(can_publish && snapshot.valid);
                    }
                }
            }
            {
                std::lock_guard<std::mutex> lock(publication_mutex);
                const bool valid_clock = clock.valid();
                clock_valid.store(valid_clock);
                if (!valid_clock || imu.session() != session || !estimator.publishable()) {
                    publishable.store(false);
                    snapshot.valid = false;
                }
            }
            saveStats();
            if (!did_work) {
                std::unique_lock<std::mutex> lock(idle_mutex);
                idle_cv.wait_for(lock, std::chrono::milliseconds(1), [&] { return stop.load(); });
            }
        }
    } catch (const std::exception& e) {
        st.last_error = e.what();
        startup(st.last_error);
    } catch (...) {
        st.last_error = "Unknown exception in VIO worker";
        startup(st.last_error);
    }
    invalidate();
    saveStats();
    running.store(false);
}

VioRuntime::VioRuntime(const VioEstimator::Options& o, VioImuSource& i, VioFrameSource& f, const VioPublicationPolicy& p) : impl_(new Impl(o, i, f, p)) {
}
VioRuntime::~VioRuntime() {
    stop();
}
bool VioRuntime::start(std::string* error) {
    if (impl_->worker.joinable()) {
        if (error) *error = "VioRuntime is already started";
        return false;
    }
    impl_->stop.store(false);
    impl_->invalidate();
    impl_->startup_done = false;
    impl_->startup_error.clear();
    {
        std::lock_guard<std::mutex> lock(impl_->stats_mutex);
        impl_->cached_stats = {};
    }
    try {
        impl_->running.store(true);
        impl_->worker = std::thread([this] { impl_->run(); });
    } catch (const std::exception& e) {
        impl_->running.store(false);
        if (error) *error = e.what();
        return false;
    }
    std::unique_lock<std::mutex> lock(impl_->startup_mutex);
    impl_->startup_cv.wait(lock, [&] { return impl_->startup_done; });
    if (!impl_->startup_error.empty()) {
        if (error) *error = impl_->startup_error;
        lock.unlock();
        stop();
        return false;
    }
    return true;
}
void VioRuntime::stop() {
    impl_->stop.store(true);
    impl_->idle_cv.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
    std::lock_guard<std::mutex> lock(impl_->publication_mutex);
    impl_->publishable.store(false);
    impl_->clock_valid.store(false);
    impl_->snapshot.valid = false;
}
bool VioRuntime::latestPose(VioPoseSnapshot& out) const {
    std::lock_guard<std::mutex> lock(impl_->publication_mutex);
    out = impl_->snapshot;
    return out.valid;
}
bool VioRuntime::publishable() const {
    return impl_->publishable.load();
}
bool VioRuntime::clockValid() const {
    return impl_->clock_valid.load();
}
uint64_t VioRuntime::timebaseGeneration() const {
    return impl_->generation.load();
}
bool VioRuntime::running() const {
    return impl_->running.load();
}
VioRuntime::Stats VioRuntime::stats() const {
    std::lock_guard<std::mutex> lock(impl_->stats_mutex);
    return impl_->cached_stats;
}
VioRuntime::PublicationGuard::PublicationGuard(const VioRuntime& r) : runtime_(r), lock_(r.impl_->publication_mutex) {
}
const VioPoseSnapshot& VioRuntime::PublicationGuard::snapshot() const {
    return runtime_.impl_->snapshot;
}
bool VioRuntime::PublicationGuard::publishable() const {
    return runtime_.impl_->publishable.load();
}
bool VioRuntime::PublicationGuard::clockValid() const {
    return runtime_.impl_->clock_valid.load();
}
bool VioRuntime::PublicationGuard::hostToFcuUs(int64_t host_ns, uint64_t& boot_us) const {
    return clockValid() && runtime_.impl_->clock.hostToFcuUs(host_ns, boot_us);
}
uint64_t VioRuntime::PublicationGuard::generation() const {
    return runtime_.impl_->generation.load();
}
VioRuntime::PublicationGuard VioRuntime::publication() const {
    return PublicationGuard(*this);
}
}  // namespace pure_uav
