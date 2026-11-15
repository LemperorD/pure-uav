// =============================================================================
//  guidance_flight_log.cpp —— 制导中间量日志实现
//
//  推导与格式都在这里；唯一的 I/O 是 record()/event() 里的 printf/fprintf。
//  设计说明见 guidance_flight_log.hpp 顶部注释与 assets/docs/app/blackbox.md。
// =============================================================================

#include "guidance_flight_log.hpp"

#include <cmath>
#include <cstdio>

namespace pure_uav::guidance::flight {
namespace {

// CSV 表头（列序固定；加列请同时改 guidance_log_csv_row 与文档）。
constexpr const char* kCsvHeader =
    "seq,frame,boot_us,mode,valid,sat,health,law,N,a_max,"
    "a_n,a_e,a_d,anorm,t_go,Vc,R,"
    "los_n,los_e,los_d,dlos_n,dlos_e,dlos_d,omega_n,omega_e,omega_d,w_raw,dt_los_ms,"
    "lam_dot,law_fallback";

bool finite_vec(const Vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

unsigned long long u64(std::uint64_t v) noexcept {
    return static_cast<unsigned long long>(v);
}

}  // namespace

GuidanceLogDerived guidance_log_derive(const GuidanceOutput& cur, const GuidanceOutput& prev, bool have_prev) {
    GuidanceLogDerived d;
    if (!have_prev) {
        return d;
    }
    if (cur.boot_us <= prev.boot_us) {
        return d;  // 时间倒退/相同：没有可用差分
    }
    const double dt_s = static_cast<double>(cur.boot_us - prev.boot_us) * 1e-6;
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
        return d;
    }
    if (!finite_vec(cur.los_hat_ned) || !finite_vec(prev.los_hat_ned)) {
        return d;
    }
    if (!(norm(cur.los_hat_ned) > 1e-9) || !(norm(prev.los_hat_ned) > 1e-9)) {
        return d;  // 无效拍的 LOS 是零矢量，不能拿来做差分
    }

    const Vec3 dlos = cur.los_hat_ned - prev.los_hat_ned;
    d.dlos = dlos;
    d.dt_s = dt_s;
    d.w_raw = norm(dlos) / dt_s;
    // ω = û × (dû/dt)；叉乘对旋转不变，NED 下与内核 ENU 下数值一致。
    d.omega = cross(cur.los_hat_ned, dlos * (1.0 / dt_s));
    d.have_prev = true;
    return d;
}

const char* guidance_mode_name(PureGuidanceMode mode) {
    switch (mode) {
        case PureGuidanceMode::PN:
            return "PN";
        case PureGuidanceMode::APN:
            return "APN";
        case PureGuidanceMode::HOLD:
            return "HOLD";
        case PureGuidanceMode::SEARCH:
            return "SEARCH";
    }
    return "?";
}

const char* guidance_health_name(GuidanceInputHealth health) {
    switch (health) {
        case GuidanceInputHealth::kOk:
            return "ok";
        case GuidanceInputHealth::kNotFinite:
            return "not_finite";
        case GuidanceInputHealth::kLosZero:
            return "los_zero";
        case GuidanceInputHealth::kLosNotUnit:
            return "los_not_unit";
        case GuidanceInputHealth::kQuatNotUnit:
            return "quat_not_unit";
        case GuidanceInputHealth::kRangeNegative:
            return "range_negative";
        case GuidanceInputHealth::kStale:
            return "stale";
        case GuidanceInputHealth::kTimeBackwards:
            return "time_backwards";
        case GuidanceInputHealth::kTimeUnknown:
            return "time_unknown";
        case GuidanceInputHealth::kNoRange:
            return "no_range";
        case GuidanceInputHealth::kNoAttitude:
            return "no_attitude";
    }
    return "?";
}

const char* guidance_law_name(GuidanceLaw law) {
    switch (law) {
        case GuidanceLaw::PPN:
            return "PPN";
        case GuidanceLaw::TPN:
            return "TPN";
        case GuidanceLaw::APN:
            return "APN";
    }
    return "?";
}

const char* guidance_log_csv_header() {
    return kCsvHeader;
}

std::string guidance_log_line(const GuidanceOutput& out, const GuidanceLogDerived& d, std::uint64_t seq, std::uint64_t frame, const GuidanceParams& params, const GuidanceLoopStats& stats) {
    char buf[512];
    const double dt_ms = d.dt_s * 1e3;
    std::snprintf(buf, sizeof(buf), "[GD] seq=%llu fr=%llu t=%.3f %s%s a=(%.3f,%.3f,%.3f)|%.2f| tgo=%.3f Vc=%.3f R=%.3f los=(%.4f,%.4f,%.4f) dlos=(%.2e,%.2e,%.2e) om=(%.4f,%.4f,%.4f) w=%.4f dt=%.1fms fb=%llu", u64(seq), u64(frame), static_cast<double>(out.boot_us) * 1e-6,
                  guidance_mode_name(out.mode), out.sat ? " SAT" : "", out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z, out.a_cmd_norm, out.t_go, out.Vc, out.R, out.los_hat_ned.x, out.los_hat_ned.y, out.los_hat_ned.z, d.dlos.x, d.dlos.y, d.dlos.z, d.omega.x, d.omega.y, d.omega.z, d.w_raw, dt_ms,
                  u64(stats.law_fallback));
    std::string line(buf);
    if (out.health != GuidanceInputHealth::kOk) {
        line += " health=";
        line += guidance_health_name(out.health);
    }
    line += " law=";
    line += guidance_law_name(params.law);
    return line;
}

std::string guidance_log_csv_row(const GuidanceOutput& out, const GuidanceLogDerived& d, std::uint64_t seq, std::uint64_t frame, const GuidanceParams& params, const GuidanceLoopStats& stats) {
    char buf[768];
    std::snprintf(buf, sizeof(buf),
                  "%llu,%llu,%llu,%d,%d,%d,%d,%s,%.9g,%.9g,"
                  "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                  "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                  "%.9g,%llu",
                  u64(seq), u64(frame), u64(out.boot_us), static_cast<int>(out.mode), out.valid ? 1 : 0, out.sat ? 1 : 0, static_cast<int>(out.health), guidance_law_name(params.law), params.N, params.a_max, out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z, out.a_cmd_norm, out.t_go, out.Vc, out.R,
                  out.los_hat_ned.x, out.los_hat_ned.y, out.los_hat_ned.z, d.dlos.x, d.dlos.y, d.dlos.z, d.omega.x, d.omega.y, d.omega.z, d.w_raw, d.dt_s * 1e3, out.lam_dot, u64(stats.law_fallback));
    return std::string(buf);
}

GuidanceFlightLog::GuidanceFlightLog(const GuidanceLogConfig& config) : config_(config) {
    enabled_ = config_.enabled && config_.rate_hz > 0.0;
    if (!enabled_) {
        return;
    }
    const double period_us = 1e6 / config_.rate_hz;
    const long long rounded = std::llround(period_us);
    period_us_ = (rounded < 1) ? 1 : static_cast<std::uint64_t>(rounded);
}

GuidanceFlightLog::~GuidanceFlightLog() {
    close();
}

bool GuidanceFlightLog::init(std::string& error) {
    error.clear();
    if (!enabled_ || config_.file.empty()) {
        return true;
    }
    file_ = std::fopen(config_.file.c_str(), "ab");
    if (file_ == nullptr) {
        error = "打不开日志文件：" + config_.file;
        return false;
    }
    std::setvbuf(file_, nullptr, _IOLBF, 0);
    std::fseek(file_, 0, SEEK_END);
    const long size = std::ftell(file_);
    if (size == 0) {
        std::fprintf(file_, "%s\n", kCsvHeader);
    }
    return true;
}

void GuidanceFlightLog::event(const char* what) {
    if (!enabled_ || what == nullptr) {
        return;
    }
    std::printf("[GD] -- %s --\n", what);
    std::fflush(stdout);
}

void GuidanceFlightLog::record(const GuidanceOutput& out, const GuidanceParams& params, const GuidanceLoopStats& stats) {
    if (!enabled_) {
        return;
    }
    if (out.boot_us == 0) {
        return;  // 没有时间基：这一拍没有可对齐的数据
    }
    if (last_boot_us_ != 0 && out.boot_us == last_boot_us_) {
        return;  // 零阶保持重发帧：没有新信息，也不能参与差分
    }
    if (last_boot_us_ != 0 && out.boot_us < last_boot_us_) {
        have_prev_ = false;  // 时间倒退：差分从这一拍重新开始
    }

    const bool computed = out.valid && out.health == GuidanceInputHealth::kOk;
    const GuidanceLogDerived d = guidance_log_derive(out, prev_, have_prev_ && computed);

    prev_ = out;
    last_boot_us_ = out.boot_us;
    have_prev_ = computed;

    ++frame_;

    // 时间抽稀：按目标帧时戳控制每秒行数（与计算拍频率无关）。
    if (last_log_us_ != 0) {
        if (out.boot_us < last_log_us_) {
            last_log_us_ = 0;  // 时间倒退：重新开始计时
        } else if (out.boot_us - last_log_us_ < period_us_) {
            return;
        }
    }
    last_log_us_ = out.boot_us;
    ++seq_;

    const std::string line = guidance_log_line(out, d, seq_, frame_, params, stats);
    std::printf("%s\n", line.c_str());
    std::fflush(stdout);
    if (file_ != nullptr) {
        const std::string row = guidance_log_csv_row(out, d, seq_, frame_, params, stats);
        std::fprintf(file_, "%s\n", row.c_str());
    }
}

void GuidanceFlightLog::close() {
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

}  // namespace pure_uav::guidance::flight
