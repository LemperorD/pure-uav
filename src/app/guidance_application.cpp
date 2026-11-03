// =============================================================================
//  guidance_application.cpp —— 启动层实现（YAML 子集 + CLI + 模式分发）
//
//  只在启动阶段调用；不用异常；不引入 yaml-cpp。
//  CLI/YAML/跑法见 assets/docs/app/usage.md。
// =============================================================================

#include "guidance_application.hpp"

#include <cctype>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "guidance_sim_tools.hpp"   // guidance_run_synthetic / guidance_run_replay / guidance_demo_camera / GuidanceSimOptions

namespace pure_uav {
namespace app {

namespace {

// ---------------------------------------------------------------- 小工具
std::string trim(const std::string &s) {
    std::size_t b = 0;
    std::size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b])) != 0) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])) != 0) --e;
    return s.substr(b, e - b);
}

std::string lower(std::string s) {
    for (char &c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool parse_double(const std::string &s, double *out) {
    if (s.empty() || out == nullptr) return false;
    errno = 0;
    char *end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

bool parse_int(const std::string &s, int *out) {
    if (s.empty() || out == nullptr) return false;
    errno = 0;
    char *end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0' || errno == ERANGE || v < INT_MIN || v > INT_MAX) {
        return false;
    }
    *out = static_cast<int>(v);
    return true;
}

bool parse_bool(const std::string &s, bool *out) {
    const std::string v = lower(s);
    if (v == "true" || v == "yes" || v == "on" || v == "1") {
        *out = true;
        return true;
    }
    if (v == "false" || v == "no" || v == "off" || v == "0") {
        *out = false;
        return true;
    }
    return false;
}

std::string err_at(const char *source, int line, const std::string &msg) {
    std::ostringstream os;
    os << (source == nullptr ? "<yaml>" : source);
    if (line > 0) os << ":" << line;
    os << ": " << msg;
    return os.str();
}

// ---------------------------------------------------------------- 枚举名
const char *law_name(GuidanceLaw v) {
    switch (v) {
        case GuidanceLaw::PPN: return "PPN";
        case GuidanceLaw::TPN: return "TPN";
        case GuidanceLaw::APN: return "APN";
    }
    return "?";
}

const char *freeze_name(FreezeCriterion v) {
    switch (v) {
        case FreezeCriterion::BY_TGO: return "BY_TGO";
        case FreezeCriterion::BY_RANGE: return "BY_RANGE";
        case FreezeCriterion::BY_TGO_OR_RANGE: return "BY_TGO_OR_RANGE";
    }
    return "?";
}

const char *los_name(LosRateSource v) {
    switch (v) {
        case LosRateSource::FROM_RELATIVE_VELOCITY: return "FROM_RELATIVE_VELOCITY";
        case LosRateSource::FROM_LOS_DIFFERENCE: return "FROM_LOS_DIFFERENCE";
    }
    return "?";
}

const char *unhealthy_name(flight::LoopConfig::OnUnhealthy v) {
    switch (v) {
        case flight::LoopConfig::OnUnhealthy::kHoldLastCommand: return "HOLD_LAST_COMMAND";
        case flight::LoopConfig::OnUnhealthy::kSafeHover: return "SAFE_HOVER";
        case flight::LoopConfig::OnUnhealthy::kStop: return "STOP";
    }
    return "?";
}

// ---------------------------------------------------------------- 键解析
bool set_enum_law(const std::string &raw, GuidanceLaw *out) {
    const std::string v = lower(trim(raw));
    if (v == "0" || v == "ppn") { *out = GuidanceLaw::PPN; return true; }
    if (v == "1" || v == "tpn") { *out = GuidanceLaw::TPN; return true; }
    if (v == "2" || v == "apn") { *out = GuidanceLaw::APN; return true; }
    return false;
}

bool set_enum_freeze(const std::string &raw, FreezeCriterion *out) {
    const std::string v = lower(trim(raw));
    if (v == "0" || v == "by_tgo" || v == "tgo") {
        *out = FreezeCriterion::BY_TGO;
        return true;
    }
    if (v == "1" || v == "by_range" || v == "range") {
        *out = FreezeCriterion::BY_RANGE;
        return true;
    }
    if (v == "2" || v == "by_tgo_or_range" || v == "tgo_or_range") {
        *out = FreezeCriterion::BY_TGO_OR_RANGE;
        return true;
    }
    return false;
}

bool set_enum_los(const std::string &raw, LosRateSource *out) {
    const std::string v = lower(trim(raw));
    if (v == "0" || v == "from_relative_velocity" || v == "relative_velocity" || v == "truth") {
        *out = LosRateSource::FROM_RELATIVE_VELOCITY;
        return true;
    }
    if (v == "1" || v == "from_los_difference" || v == "los_difference" || v == "difference") {
        *out = LosRateSource::FROM_LOS_DIFFERENCE;
        return true;
    }
    return false;
}

bool set_enum_unhealthy(const std::string &raw, flight::LoopConfig::OnUnhealthy *out) {
    const std::string v = lower(trim(raw));
    if (v == "0" || v == "hold" || v == "hold_last" || v == "hold_last_command") {
        *out = flight::LoopConfig::OnUnhealthy::kHoldLastCommand;
        return true;
    }
    if (v == "1" || v == "safe_hover" || v == "hover") {
        *out = flight::LoopConfig::OnUnhealthy::kSafeHover;
        return true;
    }
    if (v == "2" || v == "stop") {
        *out = flight::LoopConfig::OnUnhealthy::kStop;
        return true;
    }
    return false;
}

bool set_camera_matrix(const std::string &raw, double out[9]) {
    std::vector<std::string> parts;
    std::string cur;
    for (char c : raw) {
        if (c == ',') {
            parts.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    parts.push_back(cur);
    if (parts.size() != 9) return false;
    for (std::size_t i = 0; i < 9; ++i) {
        double v = 0.0;
        if (!parse_double(trim(parts[i]), &v)) return false;
        out[i] = v;
    }
    return true;
}

bool apply_key(const std::string &key, const std::string &raw, GuidanceAppParams *p, std::string *why) {
    const std::string val = trim(raw);
    auto bad = [&](const char *expect) {
        *why = "键 '" + key + "' 的值非法（期望 " + expect + "）：'" + val + "'";
        return false;
    };
    auto dbl = [&](double *dst, const char *expect) {
        if (!parse_double(val, dst)) return bad(expect);
        return true;
    };
    auto integer = [&](int *dst, const char *expect) {
        if (!parse_int(val, dst)) return bad(expect);
        return true;
    };
    auto flag = [&](bool *dst) {
        if (!parse_bool(val, dst)) return bad("true/false 或 0/1");
        return true;
    };

    if (key == "guidance.law") {
        if (!set_enum_law(val, &p->guidance.law)) return bad("PPN/TPN/APN 或 0/1/2");
        return true;
    }
    if (key == "guidance.N") return dbl(&p->guidance.N, "浮点数");
    if (key == "guidance.a_max") return dbl(&p->guidance.a_max, "浮点数 (m/s²)");
    if (key == "guidance.freeze_criterion") {
        if (!set_enum_freeze(val, &p->guidance.freeze_criterion)) {
            return bad("BY_TGO/BY_RANGE/BY_TGO_OR_RANGE 或 0/1/2");
        }
        return true;
    }
    if (key == "guidance.t_go_freeze") return dbl(&p->guidance.t_go_freeze, "浮点数 (s)");
    if (key == "guidance.R_freeze") return dbl(&p->guidance.R_freeze, "浮点数 (m)");
    if (key == "guidance.los_rate_source") {
        if (!set_enum_los(val, &p->guidance.los_rate_source)) {
            return bad("FROM_RELATIVE_VELOCITY/FROM_LOS_DIFFERENCE 或 0/1");
        }
        return true;
    }
    if (key == "loop.loop_hz") return integer(&p->loop.loop_hz, "整数 (Hz)");
    if (key == "loop.max_sample_age_s") return dbl(&p->loop.max_sample_age_s, "浮点数 (s)");
    if (key == "loop.sensor_timeout_s") return dbl(&p->loop.sensor_timeout_s, "浮点数 (s)");
    if (key == "loop.on_unhealthy") {
        if (!set_enum_unhealthy(val, &p->loop.on_unhealthy)) {
            return bad("HOLD_LAST_COMMAND/SAFE_HOVER/STOP 或 0/1/2");
        }
        return true;
    }
    if (key == "loop.hold_last_max_s") return dbl(&p->loop.hold_last_max_s, "浮点数 (s)");
    if (key == "loop.full_check_hz") return dbl(&p->loop.full_check_hz, "浮点数 (Hz)");
    if (key == "loop.send_on_every_tick") return flag(&p->loop.send_on_every_tick);
    if (key == "cam.fx") return dbl(&p->loop.cam.fx, "浮点数 (px)");
    if (key == "cam.fy") return dbl(&p->loop.cam.fy, "浮点数 (px)");
    if (key == "cam.cx") return dbl(&p->loop.cam.cx, "浮点数 (px)");
    if (key == "cam.cy") return dbl(&p->loop.cam.cy, "浮点数 (px)");
    if (key == "cam.R_body_cam") {
        if (!set_camera_matrix(val, p->loop.cam.R_body_cam)) {
            return bad("9 个逗号分隔的浮点数（行主序 R_body_cam）");
        }
        return true;
    }

    *why = "未知参数键：'" + key + "'";
    return false;
}

std::string check_range(const char *key, double v, double lo, double hi, bool lo_open,
                        bool hi_open) {
    const bool bad_lo = lo_open ? (v <= lo) : (v < lo);
    const bool bad_hi = hi_open ? (v >= hi) : (v > hi);
    if (!bad_lo && !bad_hi && std::isfinite(v)) return std::string();
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s 超出范围：%.6g（要求 %s%.6g, %.6g%s）", key, v,
                  lo_open ? "(" : "[", lo, hi, hi_open ? ")" : "]");
    return std::string(buf);
}

// ---------------------------------------------------------------- CLI
struct Args {
    int    hz    = 50;
    int    law   = 0;        // 0=PPN 1=TPN 2=APN
    double N     = 4.0;
    double a_max = 10.0;
    int    ticks = 300;
    bool   verbose = true;
    bool   realtime = false;
    const char *record = nullptr;
    const char *out    = nullptr;
    const char *replay = nullptr;
    const char *config = nullptr;
    bool   print_params = false;
    const char *blackbox = nullptr;
    double blackbox_sec = 5.0;
    bool   help = false;
    bool   law_given = false;
    bool   n_given = false;
    bool   a_max_given = false;
    bool   hz_given = false;
};

bool cli_parse_int(const char *s, int &out) {
    char *end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0' || v < 0) return false;
    out = static_cast<int>(v);
    return true;
}

bool cli_parse_double(const char *s, double &out) {
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

GuidanceLaw law_from_int(int v) {
    return (v == 1) ? GuidanceLaw::TPN : (v == 2) ? GuidanceLaw::APN : GuidanceLaw::PPN;
}

void usage(const char *prog) {
    std::printf(
        "用法: %s [选项]\n"
        "  --hz=N        主循环频率（默认 50）\n"
        "  --law=N       0=PPN 1=TPN 2=APN（默认 0）\n"
        "  --n=X         导航比（默认 4）\n"
        "  --a-max=X     过载上限 m/s²（默认 10）\n"
        "  --ticks=N     总拍数（默认 300）\n"
        "  --realtime    按真实节拍跑（默认全速）\n"
        "  --quiet       只打汇总\n"
        "  --record=F    把每拍输入录成 CSV（合成世界里也能录，便于试格式）\n"
        "  --out=F       把每拍指令落盘成 CSV\n"
        "  --replay=F    回放录制好的 CSV（不跑合成世界）\n"
        "  --config=F    YAML 参数文件（默认值 < 文件 < 命令行）\n"
        "  --print-params 打印最终参数后退出\n"
        "  --blackbox=F  跑完把黑匣子（最近 --blackbox-sec 秒）写成 CSV\n"
        "  --blackbox-sec=S  黑匣子窗口秒数（默认 5；<=0 = 全部）\n"
        "  --help        本帮助\n",
        prog);
}

bool parse_args(int argc, char **argv, Args *a) {
    for (int i = 1; i < argc; ++i) {
        const char *s = argv[i];
        auto eq = [s](const char *key) { return std::strncmp(s, key, std::strlen(key)) == 0; };
        if (eq("--hz=")) {
            if (!cli_parse_int(s + 5, a->hz)) { std::printf("!! --hz 解析失败：%s\n", s); return false; }
            a->hz_given = true;
        } else if (eq("--law=")) {
            if (!cli_parse_int(s + 6, a->law)) { std::printf("!! --law 解析失败：%s\n", s); return false; }
            a->law_given = true;
        } else if (eq("--n=")) {
            if (!cli_parse_double(s + 4, a->N)) { std::printf("!! --n 解析失败：%s\n", s); return false; }
            a->n_given = true;
        } else if (eq("--a-max=")) {
            if (!cli_parse_double(s + 8, a->a_max)) {
                std::printf("!! --a-max 解析失败：%s\n", s);
                return false;
            }
            a->a_max_given = true;
        } else if (eq("--ticks=")) {
            if (!cli_parse_int(s + 8, a->ticks)) { std::printf("!! --ticks 解析失败：%s\n", s); return false; }
        } else if (eq("--record=")) {
            a->record = s + 9;
        } else if (eq("--out=")) {
            a->out = s + 6;
        } else if (eq("--replay=")) {
            a->replay = s + 9;
        } else if (eq("--config=")) {
            a->config = s + 9;
        } else if (eq("--blackbox=")) {
            a->blackbox = s + 11;
        } else if (eq("--blackbox-sec=")) {
            if (!cli_parse_double(s + 15, a->blackbox_sec)) {
                std::printf("!! --blackbox-sec 解析失败：%s\n", s);
                return false;
            }
        } else if (std::strcmp(s, "--print-params") == 0) {
            a->print_params = true;
        } else if (std::strcmp(s, "--realtime") == 0) {
            a->realtime = true;
        } else if (std::strcmp(s, "--quiet") == 0) {
            a->verbose = false;
        } else if (std::strcmp(s, "--help") == 0 || std::strcmp(s, "-h") == 0) {
            usage(argv[0]);
            a->help = true;
            return true;
        } else {
            std::printf("未知参数：%s（--help 看用法）\n", s);
            return false;
        }
    }
    return true;
}

}  // namespace

// ============================================================================
//  参数实现（原 params.cpp）
// ============================================================================

GuidanceAppParams guidance_default_params() {
    GuidanceAppParams p;
    p.guidance.law = GuidanceLaw::PPN;
    p.guidance.N = 4.0;
    p.guidance.a_max = 10.0;
    p.guidance.freeze_criterion = FreezeCriterion::BY_TGO;
    p.guidance.t_go_freeze = 0.15;
    p.guidance.R_freeze = 5.0;
    p.guidance.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    p.loop.loop_hz = 50;
    p.loop.max_sample_age_s = 0.2;
    return p;
}

GuidanceParamResult guidance_load_yaml_text(const std::string &text, GuidanceAppParams *out, const char *source) {
    if (out == nullptr) return GuidanceParamResult::fail("内部错误：out 为空");

    GuidanceAppParams p = *out;
    std::vector<std::pair<int, std::string>> stack;
    std::set<std::string> seen;
    std::istringstream in(text);
    std::string raw;
    int line_no = 0;

    while (std::getline(in, raw)) {
        ++line_no;
        if (!raw.empty() && raw.back() == '\r') raw.pop_back();

        std::size_t first = 0;
        while (first < raw.size() && raw[first] == ' ') ++first;
        if (first < raw.size() && raw[first] == '\t') {
            return GuidanceParamResult::fail(err_at(source, line_no, "不允许用 tab 缩进（请用空格）"));
        }
        const int indent = static_cast<int>(first);

        std::string line = raw;
        bool sq = false;
        bool dq = false;
        for (std::size_t k = 0; k < line.size(); ++k) {
            const char c = line[k];
            if (c == '\'' && !dq) {
                sq = !sq;
            } else if (c == '"' && !sq) {
                dq = !dq;
            } else if (c == '#' && !sq && !dq) {
                line.resize(k);
                break;
            }
        }
        line = trim(line);
        if (line.empty()) continue;

        while (!stack.empty() && indent <= stack.back().first) stack.pop_back();

        sq = false;
        dq = false;
        std::size_t colon = std::string::npos;
        for (std::size_t k = 0; k < line.size(); ++k) {
            const char c = line[k];
            if (c == '\'' && !dq) {
                sq = !sq;
            } else if (c == '"' && !sq) {
                dq = !dq;
            } else if (c == ':' && !sq && !dq) {
                colon = k;
                break;
            }
        }
        if (colon == std::string::npos) {
            return GuidanceParamResult::fail(
                err_at(source, line_no, "缺少 ':'（本加载器只支持 `key: value`）"));
        }

        const std::string key = trim(line.substr(0, colon));
        std::string val = trim(line.substr(colon + 1));
        if (key.empty()) {
            return GuidanceParamResult::fail(err_at(source, line_no, "键名为空"));
        }
        for (char c : key) {
            if (std::isspace(static_cast<unsigned char>(c)) != 0) {
                return GuidanceParamResult::fail(err_at(source, line_no, "键名里不允许空格：'" + key + "'"));
            }
        }

        if (val.size() >= 2 && (val.front() == '"' || val.front() == '\'') &&
            val.back() == val.front()) {
            val = val.substr(1, val.size() - 2);
        }

        if (val.empty()) {
            stack.emplace_back(indent, key);
            continue;
        }

        std::string full = key;
        for (auto it = stack.rbegin(); it != stack.rend(); ++it) {
            full = it->second + "." + full;
        }
        if (!seen.insert(full).second) {
            return GuidanceParamResult::fail(err_at(source, line_no, "重复的键：'" + full + "'"));
        }

        std::string why;
        if (!apply_key(full, val, &p, &why)) {
            return GuidanceParamResult::fail(err_at(source, line_no, why));
        }
    }

    *out = p;
    return GuidanceParamResult::success();
}

GuidanceParamResult guidance_load_yaml_file(const std::string &path, GuidanceAppParams *out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return GuidanceParamResult::fail("打不开配置文件：" + path);
    std::ostringstream os;
    os << f.rdbuf();
    if (!f.good() && !f.eof()) return GuidanceParamResult::fail("读取配置文件失败：" + path);
    return guidance_load_yaml_text(os.str(), out, path.c_str());
}

GuidanceParamResult guidance_validate_params(const GuidanceAppParams &p) {
    const int law = static_cast<int>(p.guidance.law);
    if (law < 0 || law > 2) return GuidanceParamResult::fail("guidance.law 非法");
    const int freeze = static_cast<int>(p.guidance.freeze_criterion);
    if (freeze < 0 || freeze > 2) return GuidanceParamResult::fail("guidance.freeze_criterion 非法");
    const int los = static_cast<int>(p.guidance.los_rate_source);
    if (los < 0 || los > 1) return GuidanceParamResult::fail("guidance.los_rate_source 非法");
    const int unh = static_cast<int>(p.loop.on_unhealthy);
    if (unh < 0 || unh > 2) return GuidanceParamResult::fail("loop.on_unhealthy 非法");

    std::string e = check_range("guidance.N", p.guidance.N, 0.5, 20.0, true, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    e = check_range("guidance.a_max", p.guidance.a_max, 0.0, 100.0, true, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    e = check_range("guidance.t_go_freeze", p.guidance.t_go_freeze, 0.0, 10.0, false, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    e = check_range("guidance.R_freeze", p.guidance.R_freeze, 0.0, 100.0, false, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);

    if (p.loop.loop_hz < 1 || p.loop.loop_hz > 1000) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "loop.loop_hz 超出范围：%d（要求 [1, 1000]）",
                      p.loop.loop_hz);
        return GuidanceParamResult::fail(buf);
    }
    e = check_range("loop.max_sample_age_s", p.loop.max_sample_age_s, 0.0, 5.0, true, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    e = check_range("loop.sensor_timeout_s", p.loop.sensor_timeout_s, 0.0, 10.0, true, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    if (p.loop.sensor_timeout_s < p.loop.max_sample_age_s) {
        return GuidanceParamResult::fail("loop.sensor_timeout_s 必须 ≥ loop.max_sample_age_s"
                                 "（否则数据还没判过期就先判掉线）");
    }
    e = check_range("loop.hold_last_max_s", p.loop.hold_last_max_s, 0.0,
                    p.loop.sensor_timeout_s, false, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);
    e = check_range("loop.full_check_hz", p.loop.full_check_hz, 0.0, 1000.0, false, false);
    if (!e.empty()) return GuidanceParamResult::fail(e);

    const flight::CameraModel &c = p.loop.cam;
    if ((c.fx > 0.0) != (c.fy > 0.0)) {
        return GuidanceParamResult::fail("cam.fx / cam.fy 必须同时配置（现在一个 >0、一个 ≤0）");
    }
    if (c.fx < 0.0 || c.fy < 0.0) {
        return GuidanceParamResult::fail("cam.fx / cam.fy 不能为负");
    }
    if (!std::isfinite(c.cx) || !std::isfinite(c.cy)) {
        return GuidanceParamResult::fail("cam.cx / cam.cy 必须是有限数");
    }
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(c.R_body_cam[i])) {
            return GuidanceParamResult::fail("cam.R_body_cam 含非有限数");
        }
    }

    return GuidanceParamResult::success();
}

std::string guidance_params_to_string(const GuidanceAppParams &p) {
    std::ostringstream os;
    os << "guidance.law: " << law_name(p.guidance.law) << "\n";
    os << "guidance.N: " << p.guidance.N << "\n";
    os << "guidance.a_max: " << p.guidance.a_max << "\n";
    os << "guidance.freeze_criterion: " << freeze_name(p.guidance.freeze_criterion) << "\n";
    os << "guidance.t_go_freeze: " << p.guidance.t_go_freeze << "\n";
    os << "guidance.R_freeze: " << p.guidance.R_freeze << "\n";
    os << "guidance.los_rate_source: " << los_name(p.guidance.los_rate_source) << "\n";
    os << "loop.loop_hz: " << p.loop.loop_hz << "\n";
    os << "loop.max_sample_age_s: " << p.loop.max_sample_age_s << "\n";
    os << "loop.sensor_timeout_s: " << p.loop.sensor_timeout_s << "\n";
    os << "loop.on_unhealthy: " << unhealthy_name(p.loop.on_unhealthy) << "\n";
    os << "loop.hold_last_max_s: " << p.loop.hold_last_max_s << "\n";
    os << "loop.full_check_hz: " << p.loop.full_check_hz << "\n";
    os << "loop.send_on_every_tick: " << (p.loop.send_on_every_tick ? "true" : "false") << "\n";
    os << "cam.fx: " << p.loop.cam.fx << "\n";
    os << "cam.fy: " << p.loop.cam.fy << "\n";
    os << "cam.cx: " << p.loop.cam.cx << "\n";
    os << "cam.cy: " << p.loop.cam.cy << "\n";
    os << "cam.R_body_cam: ";
    for (int i = 0; i < 9; ++i) {
        if (i != 0) os << ",";
        os << p.loop.cam.R_body_cam[i];
    }
    os << "\n";
    return os.str();
}

// ============================================================================
//  GuidanceApplication
// ============================================================================

int GuidanceApplication::run(int argc, char **argv) {
    Args a;
    if (!parse_args(argc, argv, &a)) return 2;
    if (a.help) return 0;

    if (a.ticks <= 0) {
        std::printf("参数非法：--ticks 必须 > 0\n");
        return 2;
    }
    if (a.law_given && (a.law < 0 || a.law > 2)) {
        std::printf("参数非法：--law 只能是 0/1/2\n");
        return 2;
    }

    // 默认值 → YAML → 命令行（只覆盖显式给出的项）
    GuidanceAppParams ap = guidance_default_params();
    if (a.config != nullptr) {
        const GuidanceParamResult r = guidance_load_yaml_file(a.config, &ap);
        if (!r.ok) {
            std::printf("!! 配置加载失败：%s\n", r.error.c_str());
            return 2;
        }
    }
    if (a.law_given) ap.guidance.law = law_from_int(a.law);
    if (a.n_given) ap.guidance.N = a.N;
    if (a.a_max_given) ap.guidance.a_max = a.a_max;
    if (a.hz_given) ap.loop.loop_hz = a.hz;

    const GuidanceParamResult vr = guidance_validate_params(ap);
    if (!vr.ok) {
        std::printf("!! 参数校验失败：%s\n", vr.error.c_str());
        return 2;
    }

    // 合成世界 demo：配置里没给相机时回退演示相机。
    // 注意：真机必须在 YAML 里给 cam.fx/fy/cx/cy/R_body_cam。
    if (!ap.loop.cam.valid()) ap.loop.cam = guidance_demo_camera();

    if (a.print_params) {
        std::printf("%s", guidance_params_to_string(ap).c_str());
        return 0;
    }

    GuidanceSimOptions opt;
    opt.ticks        = a.ticks;
    opt.verbose      = a.verbose;
    opt.realtime     = a.realtime;
    opt.record       = a.record;
    opt.out          = a.out;
    opt.blackbox     = a.blackbox;
    opt.blackbox_sec = a.blackbox_sec;

    if (a.replay != nullptr) return guidance_run_replay(a.replay, opt, ap);
    return guidance_run_synthetic(opt, ap);
}

}  // namespace app
}  // namespace pure_uav
