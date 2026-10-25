// =============================================================================
//  params.cpp —— 参数加载 / 校验（实现）
//
//  设计约束（与 guidance_loop.hpp 的实时性规矩一致）：
//    · 只在本进程**启动阶段**调用，不参与飞行回路；
//    · 不用异常；所有失败通过 ParamResult 返回（带来源/行号）；
//    · 不引入 yaml-cpp（机载与 Windows 构建都不必为几个标量装库）。
// =============================================================================

#include "params.hpp"

#include <cctype>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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

const char *unhealthy_name(LoopConfig::OnUnhealthy v) {
    switch (v) {
        case LoopConfig::OnUnhealthy::kHoldLastCommand: return "HOLD_LAST_COMMAND";
        case LoopConfig::OnUnhealthy::kSafeHover: return "SAFE_HOVER";
        case LoopConfig::OnUnhealthy::kStop: return "STOP";
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

bool set_enum_unhealthy(const std::string &raw, LoopConfig::OnUnhealthy *out) {
    const std::string v = lower(trim(raw));
    if (v == "0" || v == "hold" || v == "hold_last" || v == "hold_last_command") {
        *out = LoopConfig::OnUnhealthy::kHoldLastCommand;
        return true;
    }
    if (v == "1" || v == "safe_hover" || v == "hover") {
        *out = LoopConfig::OnUnhealthy::kSafeHover;
        return true;
    }
    if (v == "2" || v == "stop") {
        *out = LoopConfig::OnUnhealthy::kStop;
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

// 应用一个键；成功返回 true；失败写 *why（不含来源/行号）。
bool apply_key(const std::string &key, const std::string &raw, AppParams *p, std::string *why) {
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

// ---------------------------------------------------------------- 校验
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

}  // namespace

// ============================================================
AppParams default_params() {
    AppParams p;
    // 制导：与 main.cpp 2026-10-25 之前的硬编码一致
    p.guidance.law = GuidanceLaw::PPN;
    p.guidance.N = 4.0;
    p.guidance.a_max = 10.0;
    p.guidance.freeze_criterion = FreezeCriterion::BY_TGO;
    p.guidance.t_go_freeze = 0.15;   // GuidanceParams 默认值
    p.guidance.R_freeze = 5.0;       // GuidanceParams 默认值
    // 合成世界给的是真值 ⇒ 默认真值法；真机配置里必须改 FROM_LOS_DIFFERENCE
    p.guidance.los_rate_source = LosRateSource::FROM_RELATIVE_VELOCITY;

    // 主循环：与 main.cpp 的 make_config() 一致
    p.loop.loop_hz = 50;
    p.loop.max_sample_age_s = 0.2;   // 合成世界每拍更新，放宽一点防边界误判
    // 其余 LoopConfig 字段保留默认（sensor_timeout 0.5 / hold 0.2 / full_check 2 Hz ...）
    // 相机留空：必须由配置给出；仅合成世界 demo 才回退 demo_camera()
    return p;
}

// ============================================================
ParamResult load_yaml_text(const std::string &text, AppParams *out, const char *source) {
    if (out == nullptr) return ParamResult::fail("内部错误：out 为空");

    AppParams p = *out;   // 在现有值上覆盖（默认值 → 文件 → CLI）
    std::vector<std::pair<int, std::string>> stack;   // (indent, section)
    std::set<std::string> seen;
    std::istringstream in(text);
    std::string raw;
    int line_no = 0;

    while (std::getline(in, raw)) {
        ++line_no;
        if (!raw.empty() && raw.back() == '\r') raw.pop_back();

        // 缩进（只允许空格；tab 明确报错，避免"看起来对齐、实际没对齐"）
        std::size_t first = 0;
        while (first < raw.size() && raw[first] == ' ') ++first;
        if (first < raw.size() && raw[first] == '\t') {
            return ParamResult::fail(err_at(source, line_no, "不允许用 tab 缩进（请用空格）"));
        }
        const int indent = static_cast<int>(first);

        // 去注释（引号外的 #）
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

        // 退栈：遇到不深于当前层的缩进
        while (!stack.empty() && indent <= stack.back().first) stack.pop_back();

        // 找冒号（引号外）
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
            return ParamResult::fail(
                err_at(source, line_no, "缺少 ':'（本加载器只支持 `key: value`）"));
        }

        const std::string key = trim(line.substr(0, colon));
        std::string val = trim(line.substr(colon + 1));
        if (key.empty()) {
            return ParamResult::fail(err_at(source, line_no, "键名为空"));
        }
        for (char c : key) {
            if (std::isspace(static_cast<unsigned char>(c)) != 0) {
                return ParamResult::fail(err_at(source, line_no, "键名里不允许空格：'" + key + "'"));
            }
        }

        // 去掉值两端成对的引号
        if (val.size() >= 2 && (val.front() == '"' || val.front() == '\'') &&
            val.back() == val.front()) {
            val = val.substr(1, val.size() - 2);
        }

        // 空值 = 段（缩进嵌套）
        if (val.empty()) {
            stack.emplace_back(indent, key);
            continue;
        }

        // 拼完整键
        std::string full = key;
        for (auto it = stack.rbegin(); it != stack.rend(); ++it) {
            full = it->second + "." + full;
        }
        if (!seen.insert(full).second) {
            return ParamResult::fail(err_at(source, line_no, "重复的键：'" + full + "'"));
        }

        std::string why;
        if (!apply_key(full, val, &p, &why)) {
            return ParamResult::fail(err_at(source, line_no, why));
        }
    }

    *out = p;
    return ParamResult::success();
}

// ============================================================
ParamResult load_yaml_file(const std::string &path, AppParams *out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return ParamResult::fail("打不开配置文件：" + path);
    std::ostringstream os;
    os << f.rdbuf();
    if (!f.good() && !f.eof()) return ParamResult::fail("读取配置文件失败：" + path);
    return load_yaml_text(os.str(), out, path.c_str());
}

// ============================================================
ParamResult validate(const AppParams &p) {
    // --- 枚举合法性（防止有人把整数强转成非法枚举）---
    const int law = static_cast<int>(p.guidance.law);
    if (law < 0 || law > 2) return ParamResult::fail("guidance.law 非法");
    const int freeze = static_cast<int>(p.guidance.freeze_criterion);
    if (freeze < 0 || freeze > 2) return ParamResult::fail("guidance.freeze_criterion 非法");
    const int los = static_cast<int>(p.guidance.los_rate_source);
    if (los < 0 || los > 1) return ParamResult::fail("guidance.los_rate_source 非法");
    const int unh = static_cast<int>(p.loop.on_unhealthy);
    if (unh < 0 || unh > 2) return ParamResult::fail("loop.on_unhealthy 非法");

    // --- 制导 ---
    std::string e = check_range("guidance.N", p.guidance.N, 0.5, 20.0, true, false);
    if (!e.empty()) return ParamResult::fail(e);
    e = check_range("guidance.a_max", p.guidance.a_max, 0.0, 100.0, true, false);
    if (!e.empty()) return ParamResult::fail(e);
    e = check_range("guidance.t_go_freeze", p.guidance.t_go_freeze, 0.0, 10.0, false, false);
    if (!e.empty()) return ParamResult::fail(e);
    e = check_range("guidance.R_freeze", p.guidance.R_freeze, 0.0, 100.0, false, false);
    if (!e.empty()) return ParamResult::fail(e);

    // --- 主循环 ---
    if (p.loop.loop_hz < 1 || p.loop.loop_hz > 1000) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "loop.loop_hz 超出范围：%d（要求 [1, 1000]）",
                      p.loop.loop_hz);
        return ParamResult::fail(buf);
    }
    e = check_range("loop.max_sample_age_s", p.loop.max_sample_age_s, 0.0, 5.0, true, false);
    if (!e.empty()) return ParamResult::fail(e);
    e = check_range("loop.sensor_timeout_s", p.loop.sensor_timeout_s, 0.0, 10.0, true, false);
    if (!e.empty()) return ParamResult::fail(e);
    if (p.loop.sensor_timeout_s < p.loop.max_sample_age_s) {
        return ParamResult::fail("loop.sensor_timeout_s 必须 ≥ loop.max_sample_age_s"
                                 "（否则数据还没判过期就先判掉线）");
    }
    e = check_range("loop.hold_last_max_s", p.loop.hold_last_max_s, 0.0,
                    p.loop.sensor_timeout_s, false, false);
    if (!e.empty()) return ParamResult::fail(e);
    e = check_range("loop.full_check_hz", p.loop.full_check_hz, 0.0, 1000.0, false, false);
    if (!e.empty()) return ParamResult::fail(e);

    // --- 相机：允许整块不配（合成世界 demo 会回退），但配了就必须成对且有限 ---
    const CameraModel &c = p.loop.cam;
    if ((c.fx > 0.0) != (c.fy > 0.0)) {
        return ParamResult::fail("cam.fx / cam.fy 必须同时配置（现在一个 >0、一个 ≤0）");
    }
    if (c.fx < 0.0 || c.fy < 0.0) {
        return ParamResult::fail("cam.fx / cam.fy 不能为负");
    }
    if (!std::isfinite(c.cx) || !std::isfinite(c.cy)) {
        return ParamResult::fail("cam.cx / cam.cy 必须是有限数");
    }
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(c.R_body_cam[i])) {
            return ParamResult::fail("cam.R_body_cam 含非有限数");
        }
    }

    return ParamResult::success();
}

// ============================================================
std::string to_string(const AppParams &p) {
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

}  // namespace app
}  // namespace pure_uav
