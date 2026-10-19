// MAVLink 单位换算模块单元测试 —— 王乙芃
//
// 风格：无外部测试框架，纯 main() + 轻量断言（与仓库 test_math / test_guidance 一致）。
//
// 覆盖：
//    1. 长度 / 速度（mm·cm·m、cm/s·m/s）
//    2. 角度：deg ↔ rad
//    3. 角度：cdeg（MAVLink 的 degrees*100）
//    4. 角度：degE7（MAVLink 的 lat/lon）
//    5. 角速度：deg/s ↔ rad/s
//    6. ★ 角度归一化与角差（±π 边界、358° 假跳变）
//    7. 时间：s / ms / us
//    8. ★★ uint32 毫秒回绕（约 49.7 天）—— 本模块最容易被忽略的边界
//    9. ★ 取整 / 饱和 / 环绕（写回 MAVLink 整型字段）
//   10. ★ 哨兵值（MAVLink 的 invalid 约定）
//   11. 精度：往返一致性
//
// 运行：./bin/test/test_mavlink_units   （退出码 0=全过，1=有失败）

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "mavlink_units.hpp"

using namespace pure_uav::mavlink_units;
namespace mu = pure_uav::mu;   // 顺便验证便捷别名可用

// ============================================================
//  轻量断言
// ============================================================
static int g_pass = 0;
static int g_fail = 0;

static void ok(bool cond, const char *what) {
    if (cond) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s\n", what);
    }
}

static void near(double got, double want, double tol, const char *what) {
    if (std::fabs(got - want) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got %.10g, want %.10g (tol %.2g)\n", what, got, want, tol);
    }
}

static void near_i(long long got, long long want, long long tol, const char *what) {
    const long long d = (got > want) ? (got - want) : (want - got);
    if (d <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got %lld, want %lld (tol %lld)\n", what, got, want, tol);
    }
}

static void section(const char *title) {
    std::printf("\n== %s ==\n", title);
}

// 测试自带的 π（故意不引用模块里的 PI_REF，避免"用被测量自己的常量去验证它"）
static constexpr double PI_REF = 3.14159265358979323846;

// ============================================================
//  1. 长度 / 速度
// ============================================================
static void test_length_speed() {
    section("1. 长度 / 速度");
    near(mm_to_m(1500.0), 1.5, 0.0, "mm → m");
    near(m_to_mm(1.5), 1500.0, 0.0, "m → mm");
    near(cm_to_m(150.0), 1.5, 0.0, "cm → m");
    near(m_to_cm(1.5), 150.0, 0.0, "m → cm");

    near(cm_s_to_m_s(1500.0), 15.0, 0.0, "cm/s → m/s");
    near(m_s_to_cm_s(15.0), 1500.0, 0.0, "m/s → cm/s");

    // 负值 / 零
    near(mm_to_m(-250.0), -0.25, 0.0, "负值 mm → m");
    near(cm_s_to_m_s(0.0), 0.0, 0.0, "零速度");
}

// ============================================================
//  2. 角度：deg ↔ rad
// ============================================================
static void test_deg_rad() {
    section("2. 角度：deg ↔ rad");
    near(deg_to_rad(0.0), 0.0, 0.0, "0° → 0 rad");
    near(deg_to_rad(180.0), PI_REF, 1e-15, "180° → π");
    near(deg_to_rad(90.0), PI_REF / 2.0, 1e-15, "90° → π/2");
    near(deg_to_rad(-90.0), -PI_REF / 2.0, 1e-15, "-90° → −π/2");
    near(deg_to_rad(360.0), 2.0 * PI_REF, 1e-14, "360° → 2π");

    near(rad_to_deg(PI_REF), 180.0, 1e-13, "π → 180°");
    near(rad_to_deg(PI_REF / 6.0), 30.0, 1e-13, "π/6 → 30°");
    near(rad_to_deg(0.0), 0.0, 0.0, "0 rad → 0°");

    // 往返
    const double degs[] = {0.0, 1.0, 45.0, 89.999, 123.456, -179.5, 359.99};
    for (double d : degs) {
        near(rad_to_deg(deg_to_rad(d)), d, 1e-12, "deg → rad → deg 往返");
    }
}

// ============================================================
//  3. 角度：cdeg（MAVLink 的 degrees*100）
// ============================================================
static void test_cdeg() {
    section("3. 角度：cdeg（degrees*100）");
    near(cdeg_to_deg(9000.0), 90.0, 1e-12, "9000 cdeg → 90°");
    near(cdeg_to_deg(35999.0), 359.99, 1e-11, "35999 cdeg → 359.99°");
    near(cdeg_to_rad(18000.0), PI_REF, 1e-15, "18000 cdeg → π");
    near(cdeg_to_rad(9000.0), PI_REF / 2.0, 1e-15, "9000 cdeg → π/2");

    near(rad_to_cdeg(PI_REF), 18000.0, 1e-10, "π → 18000 cdeg");
    near(deg_to_cdeg(123.45), 12345.0, 1e-11, "123.45° → 12345 cdeg");
    near(deg_to_cdeg(cdeg_to_deg(12345.0)), 12345.0, 1e-11, "cdeg → deg → cdeg 往返");
}

// ============================================================
//  4. 角度：degE7（MAVLink 的 lat/lon）
// ============================================================
static void test_deg_e7() {
    section("4. 角度：degE7（lat/lon）");
    // PX4 SITL 默认原点（苏黎世）：47.3979710°N
    near(deg_e7_to_deg(473979710.0), 47.397971, 1e-9, "473979710 degE7 → 47.397971°");
    near(deg_e7_to_deg(854616374.0), 85.4616374, 1e-9, "854616374 degE7 → 85.4616374°");
    near(deg_e7_to_deg(0.0), 0.0, 0.0, "0 degE7 → 0°");
    near(deg_e7_to_deg(-1220000000.0), -122.0, 1e-9, "负经度");

    near(deg_to_deg_e7(47.0), 470000000.0, 0.0, "47° → 470000000 degE7");
    near(deg_to_deg_e7(-122.0), -1220000000.0, 0.0, "−122° → −1220000000 degE7");

    // 往返（浮点往返会有 1 ulp 量级的抖动，容差取 1）
    near_i(deg_to_deg_e7_round(deg_e7_to_deg(473979710.0)), 473979710, 1,
           "degE7 → deg → degE7 往返");
}

// ============================================================
//  5. 角速度：deg/s ↔ rad/s
// ============================================================
static void test_angular_rate() {
    section("5. 角速度：deg/s ↔ rad/s");
    near(deg_s_to_rad_s(180.0), PI_REF, 1e-15, "180 deg/s → π rad/s");
    near(deg_s_to_rad_s(90.0), PI_REF / 2.0, 1e-15, "90 deg/s → π/2 rad/s");
    near(rad_s_to_deg_s(PI_REF), 180.0, 1e-13, "π rad/s → 180 deg/s");
    near(rad_s_to_deg_s(deg_s_to_rad_s(37.5)), 37.5, 1e-12, "deg/s → rad/s → deg/s 往返");
    near(deg_s_to_rad_s(0.0), 0.0, 0.0, "零角速度");
}

// ============================================================
//  6. 角度归一化与角差
// ============================================================
static void test_wrap() {
    section("6. 角度归一化与角差");

    near(wrap_pi(0.0), 0.0, 0.0, "wrap_pi(0) = 0");
    near(wrap_pi(PI_REF / 2.0), PI_REF / 2.0, 0.0, "wrap_pi(π/2) = π/2");
    near(wrap_pi(-PI_REF / 2.0), -PI_REF / 2.0, 0.0, "wrap_pi(−π/2) = −π/2");
    near(wrap_pi(3.0 * PI_REF / 2.0), -PI_REF / 2.0, 1e-15, "wrap_pi(3π/2) = −π/2");
    near(wrap_pi(-3.0 * PI_REF / 2.0), PI_REF / 2.0, 1e-15, "wrap_pi(−3π/2) = π/2");
    near(wrap_pi(2.0 * PI_REF), 0.0, 1e-15, "wrap_pi(2π) = 0");
    // ±π 两端点物理等价，只校验模长
    near(std::fabs(wrap_pi(PI_REF)), PI_REF, 1e-15, "|wrap_pi(π)| = π（±π 等价）");
    // π 的奇数倍落在端点，归属不保证 ⇒ 只校验模长
    near(std::fabs(wrap_pi(7.0 * PI_REF)), PI_REF, 1e-14, "|wrap_pi(7π)| = π（端点归属不保证）");
    // 大角度也要落回区间
    const double big = wrap_pi(1000.0);
    ok(big >= -PI_REF && big <= PI_REF, "wrap_pi(1000) 落在 [−π, π]");
    // 非有限值原样透传
    ok(std::isnan(wrap_pi(std::nan(""))), "wrap_pi(NaN) 仍为 NaN");
    ok(std::isinf(wrap_pi(std::numeric_limits<double>::infinity())),
       "wrap_pi(+Inf) 仍为 +Inf");

    near(wrap_2pi(0.0), 0.0, 0.0, "wrap_2pi(0) = 0");
    near(wrap_2pi(PI_REF / 2.0), PI_REF / 2.0, 0.0, "wrap_2pi(π/2) = π/2");
    near(wrap_2pi(-PI_REF / 2.0), 3.0 * PI_REF / 2.0, 1e-15, "wrap_2pi(−π/2) = 3π/2");
    near(wrap_2pi(2.0 * PI_REF), 0.0, 1e-15, "wrap_2pi(2π) = 0");
    near(wrap_2pi(3.0 * PI_REF), PI_REF, 1e-15, "wrap_2pi(3π) = π");

    // ★ 358° 假跳变：+179° 与 −179° 实际只差 2°
    //   符号约定：正值 = a 在 b 的**逆时针**方向（角度增大方向）
    near(angle_diff_pi(deg_to_rad(179.0), deg_to_rad(-179.0)), -deg_to_rad(2.0), 1e-12,
         "angle_diff_pi(179°, −179°) = −2°（顺时针 2°，不是 358°）");
    near(angle_diff_pi(deg_to_rad(-179.0), deg_to_rad(179.0)), deg_to_rad(2.0), 1e-12,
         "angle_diff_pi(−179°, 179°) = +2°（逆时针 2°）");
    near(angle_diff_pi(0.3, 0.1), 0.2, 1e-15, "angle_diff_pi 普通情形");
    near(angle_diff_pi(0.1, 0.3), -0.2, 1e-15, "angle_diff_pi 普通情形（反序）");
}

// ============================================================
//  7. 时间：s / ms / us
// ============================================================
static void test_time() {
    section("7. 时间：s / ms / us");
    near(ms_to_s(1500.0), 1.5, 0.0, "1500 ms → 1.5 s");
    near(us_to_s(1500000.0), 1.5, 0.0, "1500000 us → 1.5 s");
    near(s_to_ms(1.5), 1500.0, 0.0, "1.5 s → 1500 ms");
    near(s_to_us(1.5), 1500000.0, 0.0, "1.5 s → 1500000 us");
    near(ms_to_s(0.0), 0.0, 0.0, "0 ms");
}

// ============================================================
//  8. ★★ uint32 毫秒回绕
// ============================================================
static void test_ms_wrap() {
    section("8. uint32 毫秒回绕（约 49.7 天）");

    // 普通情形
    ok(mu::ms_diff_u32(1000u, 400u) == 600u, "普通差：1000 − 400 = 600 ms");
    ok(mu::ms_diff_u32(400u, 400u) == 0u, "同刻差 = 0");

    // ★ 回绕：prev 接近 uint32 上限，now 已绕回 0 附近
    //   prev = 4294967290，now = 15  ⇒ 真实间隔 21 ms
    ok(mu::ms_diff_u32(15u, 4294967290u) == 21u,
       "★回绕后无符号差仍正确：15 − 4294967290 = 21 ms");
    near(mu::ms_diff_to_s(15u, 4294967290u), 0.021, 1e-12, "回绕后换算成秒 = 0.021 s");

    // 超时判断（回绕安全）
    ok(mu::ms_elapsed(1000u, 400u), "ms_elapsed：已过 400 → true");
    ok(!mu::ms_elapsed(400u, 1000u), "ms_elapsed：还没到 1000 → false");
    ok(mu::ms_elapsed(400u, 400u), "ms_elapsed：恰好到达 → true");
    // ★ 跨回绕：deadline = 4294967290（旧），now = 15（已绕回）⇒ 已过
    ok(mu::ms_elapsed(15u, 4294967290u), "★ms_elapsed 跨回绕：已过 → true");
    ok(!mu::ms_elapsed(4294967290u, 15u), "★ms_elapsed 跨回绕：未到 → false");

    // 边界：恰好 2^31（半个周期）—— 临界，这里只要求不崩且语义一致
    ok(!mu::ms_elapsed(0u, 0x80000000u), "半周期边界不误判为已过");
}

// ============================================================
//  9. ★ 取整 / 饱和 / 环绕（写回 MAVLink 整型字段）
// ============================================================
static void test_rounding() {
    section("9. 取整 / 饱和 / 环绕");

    // 四舍五入（half away from zero）
    ok(round_to_i32_sat(2.4) == 2, "2.4 → 2");
    ok(round_to_i32_sat(2.5) == 3, "2.5 → 3（远离零）");
    ok(round_to_i32_sat(2.6) == 3, "2.6 → 3");
    ok(round_to_i32_sat(-2.5) == -3, "−2.5 → −3（远离零）");

    // 饱和
    ok(round_to_i32_sat(3.0e18) == std::numeric_limits<int32_t>::max(), "巨大正数 → INT32_MAX");
    ok(round_to_i32_sat(-3.0e18) == std::numeric_limits<int32_t>::min(), "巨大负数 → INT32_MIN");
    ok(round_to_i32_sat(std::numeric_limits<double>::infinity())
           == std::numeric_limits<int32_t>::max(), "+Inf → INT32_MAX");
    ok(round_to_i32_sat(std::nan("")) == 0, "NaN → 0");
    ok(round_to_u16_sat(-5.0) == 0u, "负数 → 0（uint16 下界）");
    ok(round_to_u16_sat(1.0e9) == 65535u, "巨大正数 → 65535（uint16 上界）");
    ok(round_to_u16_sat(1234.6) == 1235u, "1234.6 → 1235");

    // ★ 度 → cdeg：四舍五入 + 环绕到 [0, 36000)
    ok(deg_to_cdeg_wrapped(0.0) == 0u, "0° → 0 cdeg");
    ok(deg_to_cdeg_wrapped(123.45) == 12345u, "123.45° → 12345 cdeg");
    ok(deg_to_cdeg_wrapped(359.994) == 35999u, "359.994° → 35999 cdeg");
    ok(deg_to_cdeg_wrapped(359.999) == 0u, "★359.999° 四舍五入到 36000 ⇒ 绕回 0");
    ok(deg_to_cdeg_wrapped(360.0) == 0u, "360° → 0 cdeg");
    ok(deg_to_cdeg_wrapped(720.5) == 50u, "720.5° → 50 cdeg");
    ok(deg_to_cdeg_wrapped(-0.001) == 0u, "−0.001° → 0 cdeg（负数也绕回正区间）");
    ok(deg_to_cdeg_wrapped(-10.0) == 35000u, "−10° → 35000 cdeg");

    // ★ 度 → degE7：四舍五入 + 饱和
    ok(deg_to_deg_e7_round(47.0) == 470000000, "47° → 470000000 degE7");
    ok(deg_to_deg_e7_round(-122.0) == -1220000000, "−122° → −1220000000 degE7");
    // 200° × 1e7 = 2.0e9 < INT32_MAX(2.147e9) ⇒ 仍在范围内，不饱和
    ok(deg_to_deg_e7_round(200.0) == 2000000000, "200° 仍在 int32 范围内（不饱和）");
    // ★ 饱和阈值是 |deg| > 214.7483647
    ok(deg_to_deg_e7_round(214.7483647) == std::numeric_limits<int32_t>::max(),
       "阈值 214.7483647° 恰好取到 INT32_MAX");
    ok(deg_to_deg_e7_round(300.0) == std::numeric_limits<int32_t>::max(),
       "★300° 超出 int32 表示范围 ⇒ 饱和到 INT32_MAX");
    ok(deg_to_deg_e7_round(-300.0) == std::numeric_limits<int32_t>::min(),
       "★−300° ⇒ 饱和到 INT32_MIN");
    ok(deg_to_deg_e7_round(std::nan("")) == 0, "NaN → 0");
}

// ============================================================
//  10. ★ 哨兵值（MAVLink 的 invalid 约定）
// ============================================================
static void test_sentinels() {
    section("10. 哨兵值（invalid）");

    ok(is_valid_heading_cdeg(0u), "hdg = 0 有效");
    ok(is_valid_heading_cdeg(12345u), "hdg = 12345 有效");
    ok(!is_valid_heading_cdeg(kInvalidU16), "★hdg = UINT16_MAX(65535) ⇒ 无效");

    double out = -123.456;
    ok(!heading_cdeg_to_rad(kInvalidU16, out), "★哨兵值 → 返回 false");
    near(out, -123.456, 0.0, "★无效时 out 不被写入");
    ok(heading_cdeg_to_rad(9000u, out), "有效 hdg → 返回 true");
    near(out, PI_REF / 2.0, 1e-15, "hdg=9000 cdeg → π/2");

    double out2 = -1.0;
    ok(heading_cdeg_to_deg(12345u, out2), "有效 hdg → true");
    near(out2, 123.45, 1e-11, "hdg=12345 cdeg → 123.45°");
    ok(!heading_cdeg_to_deg(kInvalidU16, out2), "哨兵值 → false");
    near(out2, 123.45, 0.0, "第二次调用无效时 out 保持上次的值（未被写入）");

    // lat/lon 的防御性检查
    ok(is_plausible_latlon_deg_e7(473979710), "正常纬度 → plausible");
    ok(!is_plausible_latlon_deg_e7(std::numeric_limits<int32_t>::max()),
       "INT32_MAX → 判为可疑（防御性，非规范）");
    ok(!is_plausible_latlon_deg_e7(std::numeric_limits<int32_t>::min()),
       "INT32_MIN → 判为可疑（防御性，非规范）");
}

// ============================================================
//  11. 精度：往返一致性
// ============================================================
static void test_precision() {
    section("11. 精度：往返一致性");

    // deg → rad → deg：相对误差应在 1e-15 量级
    double worst = 0.0;
    for (int d = -180; d <= 180; ++d) {
        const double x = static_cast<double>(d) + 0.123456;
        const double back = rad_to_deg(deg_to_rad(x));
        const double rel = std::fabs(back - x) / (std::fabs(x) > 1.0 ? std::fabs(x) : 1.0);
        if (rel > worst) worst = rel;
    }
    ok(worst < 1e-14, "deg → rad → deg 最大相对误差 < 1e-14");

    // cdeg → rad → cdeg：量化步长 0.01° 远大于浮点误差
    worst = 0.0;
    for (int c = 0; c < 36000; c += 37) {
        const double back = rad_to_cdeg(cdeg_to_rad(static_cast<double>(c)));
        const double err = std::fabs(back - static_cast<double>(c));
        if (err > worst) worst = err;
    }
    ok(worst < 1e-8, "cdeg → rad → cdeg 最大绝对误差 < 1e-8（cdeg 量级）");

    // 一个 MAVLink 真实取值链：hdg=12345 cdeg → rad → deg
    const double hdg_rad = cdeg_to_rad(12345.0);
    near(rad_to_deg(hdg_rad), 123.45, 1e-11, "hdg 12345 cdeg → 123.45°");
}

// ============================================================
int main() {
    std::printf("============================================================\n");
    std::printf(" MAVLink 单位换算单元测试\n");
    std::printf("============================================================\n");

    test_length_speed();
    test_deg_rad();
    test_cdeg();
    test_deg_e7();
    test_angular_rate();
    test_wrap();
    test_time();
    test_ms_wrap();
    test_rounding();
    test_sentinels();
    test_precision();

    std::printf("\n----------------------------------------\n");
    std::printf("通过 %d, 失败 %d\n", g_pass, g_fail);
    std::printf("%s\n", (g_fail == 0) ? "ALL PASS" : "SOME FAILED");
    return (g_fail == 0) ? 0 : 1;
}
