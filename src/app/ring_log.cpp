// =============================================================================
//  ring_log.cpp —— 黑匣子实现（push 零 I/O；dump 才碰磁盘）
// =============================================================================

#include "ring_log.hpp"

#include <cstdio>

namespace pure_uav {
namespace app {

void RingLog::push(const GuidanceOutput &o) noexcept {
    RingRecord &r = r_[head_];
    r = RingRecord{};                       // 清零（避免上一圈残留）

    r.boot_us = o.boot_us;
    r.ctrl    = static_cast<std::uint8_t>(o.ctrl);
    r.mode    = static_cast<std::uint8_t>(o.mode);
    r.health  = static_cast<std::uint8_t>(o.health);

    unsigned f = 0;
    if (o.valid)    f |= 1u;
    if (o.sat)      f |= 2u;
    if (o.hold_yaw) f |= 4u;
    if (o.ctrl == ControlMode::kVelocity) f |= 8u;
    r.flags = static_cast<std::uint8_t>(f);

    r.a_ned[0] = o.a_cmd_ned.x;
    r.a_ned[1] = o.a_cmd_ned.y;
    r.a_ned[2] = o.a_cmd_ned.z;
    r.v_ned[0] = o.v_cmd_ned.x;
    r.v_ned[1] = o.v_cmd_ned.y;
    r.v_ned[2] = o.v_cmd_ned.z;
    r.a_norm   = o.a_cmd_norm;
    r.t_go     = o.t_go;
    r.Vc       = o.Vc;
    r.R        = o.R;
    r.lam_dot  = o.lam_dot;

    head_ = (head_ + 1) % kCap;
    ++n_;
}

const RingRecord &RingLog::at(int i) const noexcept {
    static const RingRecord kZero{};
    const int m = size();
    if (m <= 0) return kZero;
    if (i < 0) i = 0;
    if (i >= m) i = m - 1;
    const int start = (n_ < static_cast<std::uint64_t>(kCap)) ? 0 : head_;
    return r_[(start + i) % kCap];
}

const RingRecord &RingLog::newest() const noexcept { return at(size() - 1); }

long RingLog::dump_csv(const char *path, double seconds) const {
    if (path == nullptr) return -1;
    std::FILE *f = std::fopen(path, "wb");
    if (f == nullptr) return -1;

    std::fprintf(f,
                 "boot_us,ctrl,mode,health,valid,sat,hold_yaw,"
                 "a_n,a_e,a_d,v_n,v_e,v_d,anorm,t_go,Vc,R,lam_dot\n");

    const int m = size();
    if (m <= 0) {
        return (std::fclose(f) == 0) ? 0 : -1;
    }

    const std::uint64_t newest_us = at(m - 1).boot_us;
    const bool use_thresh = (seconds > 0.0);
    std::uint64_t thresh = 0;
    if (use_thresh) {
        const double span_us = seconds * 1e6;
        thresh = (span_us >= static_cast<double>(newest_us))
                     ? 0
                     : newest_us - static_cast<std::uint64_t>(span_us);
    }

    long rows = 0;
    for (int i = 0; i < m; ++i) {
        const RingRecord &r = at(i);
        if (use_thresh && r.boot_us < thresh) continue;
        std::fprintf(f,
                     "%llu,%u,%u,%u,%d,%d,%d,"
                     "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                     static_cast<unsigned long long>(r.boot_us),
                     static_cast<unsigned>(r.ctrl), static_cast<unsigned>(r.mode),
                     static_cast<unsigned>(r.health),
                     (r.flags & 1u) ? 1 : 0, (r.flags & 2u) ? 1 : 0,
                     (r.flags & 4u) ? 1 : 0,
                     r.a_ned[0], r.a_ned[1], r.a_ned[2],
                     r.v_ned[0], r.v_ned[1], r.v_ned[2],
                     r.a_norm, r.t_go, r.Vc, r.R, r.lam_dot);
        ++rows;
    }

    return (std::fclose(f) == 0) ? rows : -1;
}

}  // namespace app
}  // namespace pure_uav
