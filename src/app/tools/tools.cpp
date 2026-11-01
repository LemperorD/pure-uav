// =============================================================================
//  tools.cpp —— 地面工具箱实现（CSV 录放 + 合成世界 + 两种跑法）
//
//  CSV 用 C stdio：不抛异常、失败用 ok()/返回值表达。
//  格式与回放边界见 assets/docs/app/csv-format.md；黑匣子见 assets/docs/app/blackbox.md。
// =============================================================================

#include "tools.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "guidance_loop.hpp"
#include "quat.hpp"

namespace pure_uav {
namespace app {

namespace {

// ---- CSV 表头（`#` 开头 ⇒ 读取时自动跳过）----
const char *kInputHeader =
    "# pure-uav input v1: 一行 = 主循环一拍；ins_new/tgt_new=0 表示该拍无新帧（零阶保持）\n"
    "# tick,ins_new,ins_boot_us,has_pos,has_att,p_n,p_e,p_d,v_n,v_e,v_d,qw,qx,qy,qz,"
    "tgt_new,tgt_boot_us,u_px,v_px,has_box,w_px,h_px,conf,class_id,"
    "present,has_range,range_m,has_vT,vTn,vTe,vTd\n";

const char *kOutputHeader =
    "# pure-uav output v1: ctrl 0=加速度 1=零速度；mode 0=PN 1=APN 2=HOLD 3=SEARCH\n"
    "# boot_us,ctrl,mode,valid,sat,a_n,a_e,a_d,anorm,t_go,Vc,R,lam_dot,health\n";

inline int b2i(bool b) { return b ? 1 : 0; }

inline void strip_eol(char *s) {
    std::size_t n = std::strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ' ||
                     s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
}

int split_line(char *line, char **fields, int max_fields) {
    int n = 0;
    char *p = line;
    while (n < max_fields) {
        fields[n++] = p;
        char *comma = std::strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

bool to_double(const char *s, double &out) {
    if (!s || *s == '\0') { out = 0.0; return true; }
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

bool to_u64(const char *s, std::uint64_t &out) {
    if (!s || *s == '\0') { out = 0; return true; }
    char *end = nullptr;
    const unsigned long long v = std::strtoull(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<std::uint64_t>(v);
    return true;
}

bool to_int(const char *s, int &out) {
    if (!s || *s == '\0') { out = 0; return true; }
    char *end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<int>(v);
    return true;
}

constexpr int kInputFields = 31;

}  // namespace

// ============================================================================
//  InputRecorder / 装饰器
// ============================================================================

InputRecorder::~InputRecorder() { close(); }

bool InputRecorder::open(const char *path) {
    close();
    ok_ = false;
    tick_ = 0;
    rows_ = 0;
    ins_ = InsSample{};
    ins_new_ = false;

    if (!path || *path == '\0') return false;
    f_ = std::fopen(path, "wb");
    if (!f_) return false;
    if (std::fputs(kInputHeader, f_) < 0) {
        close();
        return false;
    }
    ok_ = true;
    return true;
}

void InputRecorder::on_ins(const InsSample &s, bool new_frame) noexcept {
    ins_ = s;
    ins_new_ = new_frame;
}

void InputRecorder::on_target(const TargetSample &s, bool new_frame) noexcept {
    if (!f_) {
        ok_ = false;
        return;
    }
    const int n = std::fprintf(
        f_,
        "%llu,%d,%llu,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%d,%llu,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%d,"
        "%d,%d,%.17g,%d,%.17g,%.17g,%.17g\n",
        static_cast<unsigned long long>(tick_), b2i(ins_new_),
        static_cast<unsigned long long>(ins_.boot_us), b2i(ins_.has_position),
        b2i(ins_.has_attitude), ins_.p.x, ins_.p.y, ins_.p.z, ins_.v.x, ins_.v.y,
        ins_.v.z, ins_.q[0], ins_.q[1], ins_.q[2], ins_.q[3], b2i(new_frame),
        static_cast<unsigned long long>(s.boot_us), s.u_px, s.v_px, b2i(s.has_box),
        s.w_px, s.h_px, s.conf, s.class_id, b2i(s.present), b2i(s.has_range),
        s.range_m, b2i(s.has_v_T), s.v_T.x, s.v_T.y, s.v_T.z);
    if (n < 0) ok_ = false;
    ++tick_;
    ++rows_;
}

void InputRecorder::close() {
    if (f_) {
        if (std::fclose(f_) != 0) ok_ = false;
        f_ = nullptr;
    }
}

bool RecordingInsSource::poll(InsSample &out) {
    const bool got = inner_.poll(out);
    rec_.on_ins(got ? out : last_, got);
    if (got) last_ = out;
    return got;
}

bool RecordingTargetSource::poll(TargetSample &out) {
    const bool got = inner_.poll(out);
    rec_.on_target(got ? out : last_, got);
    if (got) last_ = out;
    return got;
}

// ============================================================================
//  CsvReader
// ============================================================================

CsvReader::~CsvReader() { close(); }

void CsvReader::close() {
    if (f_) {
        std::fclose(f_);
        f_ = nullptr;
    }
}

bool CsvReader::open(const char *path) {
    close();
    ok_ = true;
    rows_ = 0;
    err_[0] = '\0';

    if (!path || *path == '\0') {
        std::snprintf(err_, sizeof(err_), "路径为空");
        ok_ = false;
        return false;
    }
    f_ = std::fopen(path, "rb");
    if (!f_) {
        std::snprintf(err_, sizeof(err_), "打不开文件：%s", path);
        ok_ = false;
        return false;
    }
    return true;
}

bool CsvReader::next(CsvRow &row) {
    if (!f_) return false;

    char line[1024];
    char *fields[40];
    int lineno = 0;

    while (std::fgets(line, sizeof(line), f_)) {
        ++lineno;
        strip_eol(line);
        if (line[0] == '\0' || line[0] == '#') continue;

        const int n = split_line(line, fields, 40);
        if (n < kInputFields) {
            std::snprintf(err_, sizeof(err_), "第 %d 行字段数 %d < %d", lineno, n,
                          kInputFields);
            ok_ = false;
            return false;
        }

        std::uint64_t tick = 0, ins_us = 0, tgt_us = 0;
        int ins_new = 0, has_pos = 0, has_att = 0, tgt_new = 0, has_box = 0;
        int class_id = -1, present = 0, has_range = 0, has_vt = 0;
        double p[3], v[3], q[4], uv[2], wh[2], conf = 0.0, range_m = 0.0, vt[3];

        bool bad = !to_u64(fields[0], tick) ||
                   !to_int(fields[1], ins_new) ||
                   !to_u64(fields[2], ins_us) ||
                   !to_int(fields[3], has_pos) ||
                   !to_int(fields[4], has_att) ||
                   !to_double(fields[5], p[0]) ||
                   !to_double(fields[6], p[1]) ||
                   !to_double(fields[7], p[2]) ||
                   !to_double(fields[8], v[0]) ||
                   !to_double(fields[9], v[1]) ||
                   !to_double(fields[10], v[2]) ||
                   !to_double(fields[11], q[0]) ||
                   !to_double(fields[12], q[1]) ||
                   !to_double(fields[13], q[2]) ||
                   !to_double(fields[14], q[3]) ||
                   !to_int(fields[15], tgt_new) ||
                   !to_u64(fields[16], tgt_us) ||
                   !to_double(fields[17], uv[0]) ||
                   !to_double(fields[18], uv[1]) ||
                   !to_int(fields[19], has_box) ||
                   !to_double(fields[20], wh[0]) ||
                   !to_double(fields[21], wh[1]) ||
                   !to_double(fields[22], conf) ||
                   !to_int(fields[23], class_id) ||
                   !to_int(fields[24], present) ||
                   !to_int(fields[25], has_range) ||
                   !to_double(fields[26], range_m) ||
                   !to_int(fields[27], has_vt) ||
                   !to_double(fields[28], vt[0]) ||
                   !to_double(fields[29], vt[1]) ||
                   !to_double(fields[30], vt[2]);
        if (bad) {
            std::snprintf(err_, sizeof(err_), "第 %d 行有非法数值", lineno);
            ok_ = false;
            return false;
        }

        row.tick    = tick;
        row.ins_new = (ins_new != 0);
        row.tgt_new = (tgt_new != 0);

        row.ins.boot_us      = ins_us;
        row.ins.p            = Vec3(p[0], p[1], p[2]);
        row.ins.v            = Vec3(v[0], v[1], v[2]);
        for (int i = 0; i < 4; ++i) row.ins.q[i] = q[i];
        row.ins.has_position = (has_pos != 0);
        row.ins.has_attitude = (has_att != 0);

        row.tgt.boot_us   = tgt_us;
        row.tgt.u_px      = uv[0];
        row.tgt.v_px      = uv[1];
        row.tgt.has_box   = (has_box != 0);
        row.tgt.w_px      = wh[0];
        row.tgt.h_px      = wh[1];
        row.tgt.conf      = conf;
        row.tgt.class_id  = class_id;
        row.tgt.present   = (present != 0);
        row.tgt.has_range = (has_range != 0);
        row.tgt.range_m   = range_m;
        row.tgt.has_v_T   = (has_vt != 0);
        row.tgt.v_T       = Vec3(vt[0], vt[1], vt[2]);

        ++rows_;
        return true;
    }
    return false;
}

// ============================================================================
//  CsvReplaySource
// ============================================================================

bool CsvReplaySource::advance() {
    if (!r_.next(row_)) {
        done_ = true;
        have_row_ = false;
        return false;
    }
    ++rows_;
    have_row_ = true;
    served_ins_ = false;
    served_tgt_ = false;
    return true;
}

bool CsvReplaySource::poll(InsSample &out) {
    if (!have_row_ || (served_ins_ && served_tgt_)) {
        if (!advance()) return false;
    }
    served_ins_ = true;
    if (!row_.ins_new) return false;
    out = row_.ins;
    return true;
}

bool CsvReplaySource::poll(TargetSample &out) {
    if (!have_row_ || (served_ins_ && served_tgt_)) {
        if (!advance()) return false;
    }
    served_tgt_ = true;
    if (!row_.tgt_new) return false;
    out = row_.tgt;
    return true;
}

// ============================================================================
//  CsvAccelSink
// ============================================================================

CsvAccelSink::CsvAccelSink(const char *path) {
    ok_ = false;
    if (!path || *path == '\0') return;
    f_ = std::fopen(path, "wb");
    if (!f_) return;
    if (std::fputs(kOutputHeader, f_) < 0) {
        std::fclose(f_);
        f_ = nullptr;
        return;
    }
    ok_ = true;
}

CsvAccelSink::~CsvAccelSink() { close(); }

void CsvAccelSink::close() {
    if (f_) {
        if (std::fclose(f_) != 0) ok_ = false;
        f_ = nullptr;
    }
}

bool CsvAccelSink::send(const GuidanceOutput &o) {
    if (!f_) {
        ok_ = false;
        return false;
    }
    const int n = std::fprintf(
        f_, "%llu,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",
        static_cast<unsigned long long>(o.boot_us),
        static_cast<int>(o.ctrl), static_cast<int>(o.mode), b2i(o.valid), b2i(o.sat),
        o.a_cmd_ned.x, o.a_cmd_ned.y, o.a_cmd_ned.z, o.a_cmd_norm, o.t_go, o.Vc, o.R,
        o.lam_dot, static_cast<int>(o.health));
    if (n < 0) {
        ok_ = false;
        return false;
    }
    ++rows_;
    return true;
}

// ============================================================================
//  合成世界
// ============================================================================

CameraModel demo_camera() {
    CameraModel c;
    c.fx = 600.0;
    c.fy = 600.0;
    c.cx = 320.0;   // 640×480 的主点
    c.cy = 240.0;
    const double R[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; ++i) c.R_body_cam[i] = R[i];
    return c;
}

SyntheticWorld::SyntheticWorld(const CameraModel &cam, const SceneConfig &sc)
    : cam_(cam), p_(sc.p0), v_(sc.v0), plate_(sc.plate), dt_(sc.dt), t_us_(sc.t0_us) {
    q_[0] = 1.0;
}

void SyntheticWorld::step() {
    v_ = v_ + a_cmd_ * dt_;
    p_ = p_ + v_ * dt_;
    t_us_ += static_cast<std::uint64_t>(dt_ * 1e6);
    a_cmd_ = Vec3{};
}

bool SyntheticWorld::poll(InsSample &out) {
    out.boot_us      = t_us_;
    out.p            = p_;
    out.v            = v_;
    for (int i = 0; i < 4; ++i) out.q[i] = q_[i];
    out.has_position = true;
    out.has_attitude = true;
    return true;
}

bool SyntheticWorld::poll(TargetSample &out) {
    const Vec3 d_world = plate_ - p_;
    const double R = norm(d_world);
    if (!(R > 1e-6)) return false;
    const Vec3 los_world = d_world / R;

    const Vec3 d_body = quat_rotate_vec_inv(q_, los_world);
    const Vec3 d_cam  = mat3_mul_transpose(cam_.R_body_cam, d_body);
    if (!(d_cam.z > 1e-6)) return false;

    out.boot_us   = t_us_;
    out.u_px      = cam_.cx + cam_.fx * (d_cam.x / d_cam.z);
    out.v_px      = cam_.cy + cam_.fy * (d_cam.y / d_cam.z);
    out.has_box   = false;
    out.w_px      = 0.0;
    out.h_px      = 0.0;
    out.conf      = 1.0;
    out.class_id  = 0;
    out.present   = true;
    out.has_range = true;
    out.range_m   = R;
    out.has_v_T   = true;
    out.v_T       = Vec3(0.0, 0.0, 0.0);
    return true;
}

bool SyntheticWorld::send(const GuidanceOutput &out) {
    ++sends_;
    last_ = out;
    if (out.ctrl == ControlMode::kAcceleration) {
        a_cmd_ = out.a_cmd_ned;
    } else {
        a_cmd_ = Vec3{};
    }
    if (out.sat) ++sat_;
    if (out.mode == PureGuidanceMode::HOLD) ++hold_;
    if (out.mode == PureGuidanceMode::SEARCH) ++search_;
    return true;
}

Vec3 SyntheticWorld::mat3_mul_transpose(const double R[9], const Vec3 &v) {
    return Vec3(R[0] * v.x + R[3] * v.y + R[6] * v.z,
                R[1] * v.x + R[4] * v.y + R[7] * v.z,
                R[2] * v.x + R[5] * v.y + R[8] * v.z);
}

ClosedLoopReport run_synthetic_closed_loop(SyntheticWorld &world, GuidanceLoop &loop,
                                           int ticks, TickHook hook, void *user) {
    ClosedLoopReport rep;
    for (int k = 0; k < ticks; ++k) {
        world.step();
        GuidanceOutput out;
        const LoopStatus st = loop.run_once(&out);
        if (st != LoopStatus::kOk) {
            rep.stop = st;
            rep.target_lost = (st == LoopStatus::kSensorTimeout);
            break;
        }

        const double R = world.true_range();
        if (R < rep.min_r) {
            rep.min_r = R;
            rep.min_r_tick = k;
        }
        if (hook) hook(user, k, world, out);
        ++rep.ticks;
    }
    return rep;
}

// ============================================================================
//  两种跑法（从原 main.cpp 搬来）
// ============================================================================

namespace {

const char *status_name(LoopStatus s) {
    switch (s) {
        case LoopStatus::kOk:             return "kOk";
        case LoopStatus::kNotInitialized: return "kNotInitialized";
        case LoopStatus::kStopped:        return "kStopped";
        case LoopStatus::kSensorTimeout:  return "kSensorTimeout";
        case LoopStatus::kUnhealthyInput: return "kUnhealthyInput";
        case LoopStatus::kSinkError:      return "kSinkError";
        case LoopStatus::kEstimatorReset: return "kEstimatorReset";
    }
    return "?";
}

const char *mode_name(PureGuidanceMode m) {
    switch (m) {
        case PureGuidanceMode::PN:     return "PN  ";
        case PureGuidanceMode::APN:    return "APN ";
        case PureGuidanceMode::HOLD:   return "HOLD";
        case PureGuidanceMode::SEARCH: return "SECH";
    }
    return "?   ";
}

const char *law_name(GuidanceLaw law) {
    switch (law) {
        case GuidanceLaw::PPN: return "PPN";
        case GuidanceLaw::TPN: return "TPN";
        case GuidanceLaw::APN: return "APN";
    }
    return "?";
}

struct Lat3 {
    unsigned long long p50 = 0;
    unsigned long long p95 = 0;
    unsigned long long max = 0;
};

Lat3 lat3(const LatencyStats &l, LatencyStats::Stage s) {
    Lat3 v;
    v.p50 = static_cast<unsigned long long>(l.percentile_ns(s, 0.50));
    v.p95 = static_cast<unsigned long long>(l.percentile_ns(s, 0.95));
    v.max = static_cast<unsigned long long>(l.max_ns(s));
    return v;
}

struct PrintCtx {
    int    every = 5;
    double dt    = 0.02;
};

void on_tick_print(void *user, int k, const SyntheticWorld &world, const GuidanceOutput &out) {
    const PrintCtx *ctx = static_cast<const PrintCtx *>(user);
    if (!ctx || ctx->every <= 0 || (k % ctx->every) != 0) return;
    std::printf("  %4d  %5.2f  %5.2f  %6.2f   %s  %6.3f   (%6.2f,%6.2f,%6.2f)  %s\n",
                k, k * ctx->dt, world.true_range(), out.Vc, mode_name(out.mode), out.t_go,
                out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z, out.sat ? "Y" : ".");
}

void print_banner(const SimOptions &opt, const AppParams &ap) {
    std::printf("=== pure-uav 制导主程序（合成世界，闭环）===\n");
    std::printf("  律=%s  N=%.1f  a_max=%.1f m/s²  loop=%d Hz  拍数=%d\n",
                law_name(ap.guidance.law), ap.guidance.N, ap.guidance.a_max,
                ap.loop.loop_hz, opt.ticks);
    std::printf("  起点 (0,0,-5) NED，初速 (8,0,0)，靶 (40,3,-5)\n");
    std::printf("  ⚠ 理想点质量积分（无姿态环/重力），目的在验证制导链路\n");
    if (!opt.realtime) {
        std::printf("  ⚠ 全速模式：降级计时按**墙钟**计（hold_last_max_s / sensor_timeout），\n");
        std::printf("     全速下不会走完 ⇒ 只能看链路是否跑通；要看降级行为请加 --realtime\n");
    }
    std::printf("\n");
    std::printf("   拍   t(s)   R(m)   Vc(m/s)  mode  t_go(s)  a_ned=(x, y, z)      sat\n");
}

}  // namespace

int run_synthetic(const SimOptions &opt, const AppParams &ap) {
    const int hz = (ap.loop.loop_hz > 0) ? ap.loop.loop_hz : 1;
    const double dt = 1.0 / static_cast<double>(hz);

    SceneConfig sc;
    sc.dt = dt;

    const CameraModel cam = ap.loop.cam.valid() ? ap.loop.cam : demo_camera();
    SyntheticWorld world(cam, sc);

    // ---- 录制（可选）：装饰器包在数据源外面，主循环代码一行不改 ----
    InputRecorder rec;
    const bool want_rec = (opt.record != nullptr);
    if (want_rec && !rec.open(opt.record)) {
        std::printf("!! 打不开录制文件：%s\n", opt.record);
        return 1;
    }
    RecordingInsSource    rins(world, rec);
    RecordingTargetSource rtgt(world, rec);
    IInsSource    &ins = want_rec ? static_cast<IInsSource &>(rins)
                                  : static_cast<IInsSource &>(world);
    ITargetSource &tgt = want_rec ? static_cast<ITargetSource &>(rtgt)
                                  : static_cast<ITargetSource &>(world);

    // ---- 指令落盘（可选）：一发两处 ----
    CsvAccelSink csv_sink(opt.out);
    if (opt.out && !csv_sink.ok()) {
        std::printf("!! 打不开输出文件：%s\n", opt.out);
        return 1;
    }
    TeeSink tee(world, csv_sink);
    ICommandSink &sink = opt.out ? static_cast<ICommandSink &>(tee)
                                 : static_cast<ICommandSink &>(world);

    GuidanceLoop loop(ap.guidance, ap.loop, ins, tgt, sink);
    if (!loop.init()) {
        std::printf("GuidanceLoop 初始化失败（检查相机模型 / 参数）\n");
        return 1;
    }

    print_banner(opt, ap);
    if (want_rec) std::printf("  录制 → %s\n", opt.record);
    if (opt.out)  std::printf("  落盘 → %s\n", opt.out);
    if (want_rec || opt.out) std::printf("\n");

    PrintCtx ctx;
    ctx.every = opt.verbose ? 5 : 0;
    ctx.dt    = dt;

    const ClosedLoopReport rep = run_synthetic_closed_loop(
        world, loop, opt.ticks, opt.verbose ? on_tick_print : nullptr, &ctx);

    const LoopStats &s = loop.stats();
    std::printf("\n=== 汇总 ===\n");
    if (rep.stop != LoopStatus::kOk) {
        std::printf("  终止：%s%s\n", status_name(rep.stop),
                    rep.target_lost ? "（目标离开视野，演示里属正常）" : "");
    }
    std::printf("  拍数 ticks=%llu  ins=%llu  tgt=%llu  sent=%llu  sink_err=%llu\n",
                (unsigned long long)s.ticks, (unsigned long long)s.ins_frames,
                (unsigned long long)s.tgt_frames, (unsigned long long)s.sent,
                (unsigned long long)s.sink_errors);
    std::printf("  饱和 sat=%d  冻结 hold=%d  丢失 search=%d  降级 unhealthy=%llu  safe_hover=%llu\n",
                world.sat(), world.hold(), world.search(),
                (unsigned long long)s.unhealthy, (unsigned long long)s.safe_hover);
    std::printf("  最近距离 min R = %.3f m（第 %d 拍）  末位置 (%.2f, %.2f, %.2f)\n",
                rep.min_r, rep.min_r_tick, world.pos().x, world.pos().y, world.pos().z);
    std::printf("  末速度 (%.2f, %.2f, %.2f)  max_latency=%llu us\n", world.vel().x,
                world.vel().y, world.vel().z, (unsigned long long)s.max_latency_us);
    if (s.latency.count() > 0) {
        const Lat3 poll = lat3(s.latency, LatencyStats::Stage::kPoll);
        const Lat3 proj = lat3(s.latency, LatencyStats::Stage::kProject);
        const Lat3 guid = lat3(s.latency, LatencyStats::Stage::kGuidance);
        const Lat3 send = lat3(s.latency, LatencyStats::Stage::kSend);
        const Lat3 tot  = lat3(s.latency, LatencyStats::Stage::kTotal);
        std::printf("  打点 P50/P95/max (ns)  取数 %llu/%llu/%llu  反投影 %llu/%llu/%llu\n",
                    poll.p50, poll.p95, poll.max, proj.p50, proj.p95, proj.max);
        std::printf("                        制导 %llu/%llu/%llu  发送 %llu/%llu/%llu"
                    "  总 %llu/%llu/%llu\n",
                    guid.p50, guid.p95, guid.max, send.p50, send.p95, send.max,
                    tot.p50, tot.p95, tot.max);
    }
    if (want_rec) std::printf("  录制行数 %llu（%s）\n",
                              (unsigned long long)rec.rows(), rec.ok() ? "ok" : "写失败");
    if (opt.out) std::printf("  落盘行数 %llu（%s）\n",
                             (unsigned long long)csv_sink.rows(),
                             csv_sink.ok() ? "ok" : "写失败");
    if (opt.blackbox != nullptr) {
        const long rows = loop.blackbox().dump_csv(opt.blackbox, opt.blackbox_sec);
        if (rows < 0) std::printf("  黑匣子：写失败 → %s\n", opt.blackbox);
        else std::printf("  黑匣子：%ld 行（最近 %.1f s）→ %s\n", rows, opt.blackbox_sec,
                         opt.blackbox);
    }

    if (rep.min_r < 1.0)
        std::printf("  ⇒ 链路闭环成立：PN 把最近距离压到了 1 m 以内 ✅\n");
    else
        std::printf("  ⇒ 最近距离仍 > 1 m：检查初速方向 / a_max / N\n");

    return rep.target_lost ? 0 : (rep.stop == LoopStatus::kOk ? 0 : 1);
}

int run_replay(const char *replay, const SimOptions &opt, const AppParams &ap) {
    CsvReader reader;
    if (!reader.open(replay)) {
        std::printf("!! 打不开回放文件：%s\n", replay);
        return 1;
    }

    CsvReplaySource src(reader);
    CsvAccelSink csv_sink(opt.out);
    if (opt.out && !csv_sink.ok()) {
        std::printf("!! 打不开输出文件：%s\n", opt.out);
        return 1;
    }
    NullSink null_sink;
    ICommandSink &sink = opt.out ? static_cast<ICommandSink &>(csv_sink)
                                 : static_cast<ICommandSink &>(null_sink);

    GuidanceLoop loop(ap.guidance, ap.loop, src, src, sink);
    if (!loop.init()) {
        std::printf("GuidanceLoop 初始化失败\n");
        return 1;
    }

    std::printf("=== 回放 %s ===\n", replay);
    std::printf("  ⚠ 回放全速跑：超时/保持窗口用的是本机时钟，几乎不前进 ⇒\n");
    std::printf("     这里只验「算得对不对」，降级时机要用 --realtime 或单测 §16\n");
    std::printf("\n");

    const int ticks = (opt.ticks > 0) ? opt.ticks : 1000000;
    int k = 0;
    LoopStatus stop = LoopStatus::kOk;
    for (; k < ticks; ++k) {
        const LoopStatus st = loop.run_once(nullptr);
        if (st != LoopStatus::kOk) {
            stop = st;
            break;
        }
        if (src.done()) break;
        if (opt.realtime) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(
                static_cast<long long>(1e9 / static_cast<double>((ap.loop.loop_hz > 0) ? ap.loop.loop_hz : 1))));
        }
    }

    const LoopStats &s = loop.stats();
    std::printf("\n=== 汇总 ===\n");
    if (src.error()) std::printf("  !! CSV 解析问题：%s\n", src.error());
    std::printf("  回放行数 %llu  循环拍数 %llu  ins=%llu  tgt=%llu  sent=%llu\n",
                (unsigned long long)src.rows(), (unsigned long long)s.ticks,
                (unsigned long long)s.ins_frames, (unsigned long long)s.tgt_frames,
                (unsigned long long)s.sent);
    std::printf("  饱和 %llu  冻结 %llu  丢失 %llu  降级 %llu  末状态 %s\n",
                (unsigned long long)s.saturated, (unsigned long long)s.hold,
                (unsigned long long)s.search, (unsigned long long)s.unhealthy,
                status_name(stop));
    if (opt.out) std::printf("  落盘行数 %llu（%s）\n",
                             (unsigned long long)csv_sink.rows(),
                             csv_sink.ok() ? "ok" : "写失败");
    if (opt.blackbox != nullptr) {
        const long rows = loop.blackbox().dump_csv(opt.blackbox, opt.blackbox_sec);
        if (rows < 0) std::printf("  黑匣子：写失败 → %s\n", opt.blackbox);
        else std::printf("  黑匣子：%ld 行（最近 %.1f s）→ %s\n", rows, opt.blackbox_sec,
                         opt.blackbox);
    }

    return (stop == LoopStatus::kOk || src.error() == nullptr) ? 0 : 1;
}

}  // namespace app
}  // namespace pure_uav
