// =============================================================================
//  CSV 录制 / 回放实现（格式与设计见 csv_io.hpp 头注释）
//
//  实现选择：**用 C stdio（FILE*）而不是 std::ofstream**
//    · 录制点跑在主循环里（实时路径）⇒ 不要异常、不要每次写入新分配；
//    · `fprintf` 一次系统调用级别地写进 stdio 缓冲，抖动最小；
//    · 落盘失败用返回值 + `ok()` 表达，不抛异常（机载不用异常）。
//  ⚠ 但**不要**在实飞的关键路径里靠 stdio 缓冲：fsync 抖动可能几十 ms。
//    真要上机，录制应放到独立线程 + 环形缓冲（见《后续注意问题.md》§9 待办）。
// =============================================================================

#include "csv_io.hpp"

#include <cstdlib>
#include <cstring>

namespace pure_uav {
namespace app {

namespace {

// ---- 表头（`#` 开头 ⇒ 读取时自动跳过）----
const char *kInputHeader =
    "# pure-uav input v1: 一行 = 主循环一拍；ins_new/tgt_new=0 表示该拍无新帧（零阶保持）\n"
    "# tick,ins_new,ins_boot_us,has_pos,has_att,p_n,p_e,p_d,v_n,v_e,v_d,qw,qx,qy,qz,"
    "tgt_new,tgt_boot_us,u_px,v_px,has_box,w_px,h_px,conf,class_id,"
    "present,has_range,range_m,has_vT,vTn,vTe,vTd\n";

const char *kOutputHeader =
    "# pure-uav output v1: ctrl 0=加速度 1=零速度；mode 0=PN 1=APN 2=HOLD 3=SEARCH\n"
    "# boot_us,ctrl,mode,valid,sat,a_n,a_e,a_d,anorm,t_go,Vc,R,lam_dot,health\n";

inline int b2i(bool b) { return b ? 1 : 0; }

// 去掉行尾 \r\n 与首尾空格
inline void strip_eol(char *s) {
    std::size_t n = std::strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ' ||
                     s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
}

// 按逗号切分（就地写 '\0'）
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

// 空字段当 0（便于手工编辑/删列），非空必须整段是数字
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
//  InputRecorder
// ============================================================================

InputRecorder::~InputRecorder() { close(); }

bool InputRecorder::open(const char *path) {
    close();
    ok_ = false;           // 先悲观：任何一步失败都不算 ok
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
    // 只更新"当前保持值"；真正落盘在 on_target（一拍一行）
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
    // ⚠ 每拍都要通知录制器（哪怕是零阶保持），否则行列会错位
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

CsvReader::~CsvReader() {
    if (f_) std::fclose(f_);
}

bool CsvReader::open(const char *path) {
    ok_ = true;
    rows_ = 0;
    err_[0] = '\0';
    if (!path || *path == '\0') return false;
    f_ = std::fopen(path, "rb");
    if (!f_) {
        std::snprintf(err_, sizeof(err_), "打不开文件：%s", path);
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
        if (line[0] == '\0' || line[0] == '#') continue;   // 空行 / 注释

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
    return false;   // 正常结束（或读错误，读错误由 ferror 判断，这里不区分）
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
    // 两个接口都交付过 ⇒ 才推进下一行（顺序无关，见头注释的契约）
    if (!have_row_ || (served_ins_ && served_tgt_)) {
        if (!advance()) return false;
    }
    served_ins_ = true;
    if (!row_.ins_new) return false;   // 零阶保持：由主循环沿用上一帧
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
    if (!path || *path == '\0') return;
    f_ = std::fopen(path, "wb");
    if (!f_) return;
    if (std::fputs(kOutputHeader, f_) < 0) {
        std::fclose(f_);
        f_ = nullptr;
    }
}

CsvAccelSink::~CsvAccelSink() {
    if (f_) std::fclose(f_);
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
//  NullSink（指令空实现；调试"只看算得对不对、不发指令"）
// ============================================================================

bool NullSink::send(const GuidanceOutput &) { return true; }
bool NullSink::alive() const { return true; }

}  // namespace app
}  // namespace pure_uav
