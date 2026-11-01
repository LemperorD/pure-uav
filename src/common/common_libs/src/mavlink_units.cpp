#include "mavlink_units.hpp"

#include <cmath>
#include <limits>

namespace pure_uav {
namespace mavlink_units {

// ============================================================================
//  内部工具
// ============================================================================
namespace {

// 四舍五入到最近整数（half away from zero），**先钳制再 llround**。
// 为什么不能直接 llround：C++ 标准规定，参数超出 long long 范围时 llround 是 UB。
long long round_clamped(double v, double lo, double hi) noexcept {
    if (std::isnan(v)) return 0;
    if (v <= lo) return static_cast<long long>(lo);
    if (v >= hi) return static_cast<long long>(hi);
    return std::llround(v);
}

}  // namespace

// ============================================================================
//  §1  长度
// ============================================================================
// 用"除以精确的 10 的幂"（1000.0 / 100.0 在二进制里可精确表示），
// 比"乘 1e-3 / 1e-2"少一次舍入误差。
double mm_to_m(double mm) noexcept { return mm / 1000.0; }
double m_to_mm(double m) noexcept { return m * 1000.0; }
double cm_to_m(double cm) noexcept { return cm / 100.0; }
double m_to_cm(double m) noexcept { return m * 100.0; }

// ============================================================================
//  §2  速度
// ============================================================================
double cm_s_to_m_s(double cm_s) noexcept { return cm_s / 100.0; }
double m_s_to_cm_s(double m_s) noexcept { return m_s * 100.0; }

// ============================================================================
//  §3  角度
// ============================================================================
double deg_to_rad(double deg) noexcept { return deg * kDegToRad; }
double rad_to_deg(double rad) noexcept { return rad * kRadToDeg; }

double cdeg_to_rad(double cdeg) noexcept { return cdeg * kCDegToRad; }
double rad_to_cdeg(double rad) noexcept { return rad * kRadToCDeg; }
double cdeg_to_deg(double cdeg) noexcept { return cdeg / 100.0; }
double deg_to_cdeg(double deg) noexcept { return deg * 100.0; }

// 1e7 可被 double 精确表示 ⇒ 除法得到"正确舍入"的商，优于乘 1e-7（两次舍入）
double deg_e7_to_deg(double deg_e7) noexcept { return deg_e7 / 1.0e7; }
double deg_to_deg_e7(double deg) noexcept { return deg * 1.0e7; }

// ============================================================================
//  §4  角速度
// ============================================================================
double deg_s_to_rad_s(double deg_s) noexcept { return deg_s * kDegToRad; }
double rad_s_to_deg_s(double rad_s) noexcept { return rad_s * kRadToDeg; }

// ============================================================================
//  §5  角度归一化与角差
// ============================================================================
double wrap_pi(double rad) noexcept {
    // std::remainder(x, 2π) = x − n·2π，n = round(x / 2π)，结果落在 [−π, π]。
    // 比 fmod(x + π, 2π) − π 更好：对大的 x 也能保持精度。
    if (!std::isfinite(rad)) return rad;
    return std::remainder(rad, kTwoPi);
}

double wrap_2pi(double rad) noexcept {
    if (!std::isfinite(rad)) return rad;
    double r = std::fmod(rad, kTwoPi);
    if (r < 0.0) r += kTwoPi;
    return r;
}

double angle_diff_pi(double a_rad, double b_rad) noexcept {
    // ⚠ a_rad / b_rad 极大时，相减本身会先丢精度；正常姿态/航向角不会遇到。
    return wrap_pi(a_rad - b_rad);
}

// ============================================================================
//  §6  时间
// ============================================================================
double ms_to_s(double ms) noexcept { return ms / 1000.0; }
double us_to_s(double us) noexcept { return us / 1.0e6; }
double s_to_ms(double s) noexcept { return s * 1000.0; }
double s_to_us(double s) noexcept { return s * 1.0e6; }

uint32_t ms_diff_u32(uint32_t now_ms, uint32_t prev_ms) noexcept {
    // 无符号减法在模 2^32 意义下运算：即使 now < prev（刚回绕过），结果仍正确。
    // 显式 static_cast 是为了不受整型提升影响（C++ 中负数转无符号是良定义的模运算）。
    return static_cast<uint32_t>(now_ms - prev_ms);
}

double ms_diff_to_s(uint32_t now_ms, uint32_t prev_ms) noexcept {
    return static_cast<double>(ms_diff_u32(now_ms, prev_ms)) / 1000.0;
}

bool ms_elapsed(uint32_t now_ms, uint32_t deadline_ms) noexcept {
    // "已过" ⇔ 无符号差落在"过去"那半区（< 2^31）。
    // 若 deadline 在未来（差 < 2^31 之外），结果为大数 ⇒ 返回 false。
    return static_cast<uint32_t>(now_ms - deadline_ms) < 0x80000000u;
}

// ============================================================================
//  §7  取整 / 饱和（写回 MAVLink 整型字段）
// ============================================================================
int32_t round_to_i32_sat(double v) noexcept {
    return static_cast<int32_t>(
        round_clamped(v, -2147483648.0, 2147483647.0));
}

uint16_t round_to_u16_sat(double v) noexcept {
    return static_cast<uint16_t>(round_clamped(v, 0.0, 65535.0));
}

uint16_t deg_to_cdeg_wrapped(double deg) noexcept {
    if (!std::isfinite(deg)) return 0;

    double d = std::fmod(deg, 360.0);
    if (d < 0.0) d += 360.0;              // → [0, 360)

    long long c = std::llround(d * 100.0); // → [0, 36000]
    if (c >= 36000) c -= 36000;            // ★ 359.999° 会四舍五入到 36000，必须绕回 0
    if (c < 0) c = 0;                      // 防御（上面的 fmod 已保证非负）

    return static_cast<uint16_t>(c);
}

int32_t deg_to_deg_e7_round(double deg) noexcept {
    if (std::isnan(deg)) return 0;

    const double v = deg * 1.0e7;
    // ★ |deg| > 214.7483647 时 v 超出 int32 ⇒ 必须饱和，否则 llround 后强转是未定义行为
    if (v >= 2147483647.0) return std::numeric_limits<int32_t>::max();
    if (v <= -2147483648.0) return std::numeric_limits<int32_t>::min();

    return static_cast<int32_t>(std::llround(v));
}

// ============================================================================
//  §8  哨兵值
// ============================================================================
bool is_valid_heading_cdeg(uint16_t raw_cdeg) noexcept {
    return raw_cdeg != kInvalidU16;
}

bool heading_cdeg_to_rad(uint16_t raw_cdeg, double &out_rad) noexcept {
    if (!is_valid_heading_cdeg(raw_cdeg)) return false;
    out_rad = cdeg_to_rad(static_cast<double>(raw_cdeg));
    return true;
}

bool heading_cdeg_to_deg(uint16_t raw_cdeg, double &out_deg) noexcept {
    if (!is_valid_heading_cdeg(raw_cdeg)) return false;
    out_deg = cdeg_to_deg(static_cast<double>(raw_cdeg));
    return true;
}

bool is_plausible_latlon_deg_e7(int32_t raw_deg_e7) noexcept {
    if (raw_deg_e7 == kInt32Max) return false;
    if (raw_deg_e7 == std::numeric_limits<int32_t>::min()) return false;
    return true;
}

}  // namespace mavlink_units
}  // namespace pure_uav
