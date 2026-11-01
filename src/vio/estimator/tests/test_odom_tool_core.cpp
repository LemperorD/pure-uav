// 联调工具核心逻辑测试（不依赖硬件）：
//   · 合成轨迹（odometry_sender 用）：位置/速度/航向在关键点上的值；
//   · ODOMETRY 统计（odometry_probe 用）：帧数/质量/reset 变化/坐标系/乱序/频率。
//
// 运行：./bin/test_odom_tool_core

#include <cmath>
#include <cstdio>
#include <cstring>

#include "common/mavlink.h"

#include "odom_tool_core.hpp"

namespace {
int g_failed = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("  ❌ 第 %d 行不成立: %s\n", __LINE__, #cond);    \
            ++g_failed;                                                  \
        }                                                                \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                 \
    do {                                                                      \
        const double _va = static_cast<double>(a);                            \
        const double _vb = static_cast<double>(b);                            \
        if (std::fabs(_va - _vb) > (tol)) {                                   \
            std::printf("  ❌ 第 %d 行: %s=%.6f 与 %s=%.6f 差超过 %.3g\n",      \
                        __LINE__, #a, _va, #b, _vb, (double)(tol));           \
            ++g_failed;                                                       \
        }                                                                     \
    } while (0)

}  // namespace

int main() {
    using namespace pure_uav::odom_tool;
    std::printf("===== 联调工具核心逻辑测试 =====\n");

    // ---- 1) 脚本轨迹 ----
    {
        CHECK_NEAR(scriptDuration(), 27.0, 1e-9);

        const auto p0 = sampleScript(0.0);
        CHECK_NEAR(p0.x, 0.0, 1e-6);
        CHECK_NEAR(p0.z, 0.0, 1e-6);
        CHECK_NEAR(p0.q[0], 1.0, 1e-6);

        const auto fwd = sampleScript(4.5);   // 前移中点
        CHECK_NEAR(fwd.x, 0.5, 1e-6);
        CHECK_NEAR(fwd.y, 0.0, 1e-6);
        CHECK_NEAR(fwd.vx, 1.0 / 3.0, 1e-6);
        CHECK_NEAR(fwd.vy, 0.0, 1e-6);

        const auto fwd_hold = sampleScript(6.5);  // 停住
        CHECK_NEAR(fwd_hold.x, 1.0, 1e-6);
        CHECK_NEAR(fwd_hold.vx, 0.0, 1e-6);

        const auto back = sampleScript(7.5);      // 退回中点
        CHECK_NEAR(back.x, 0.5, 1e-6);
        CHECK_NEAR(back.vx, -1.0, 1e-6);

        const auto right = sampleScript(10.5);    // 右移中点
        CHECK_NEAR(right.y, 0.3, 1e-6);
        CHECK_NEAR(right.vy, 0.2, 1e-6);

        const auto up = sampleScript(16.5);       // 上抬中点（FRD: 上为负）
        CHECK_NEAR(up.z, -0.25, 1e-6);
        CHECK_NEAR(up.vz, -0.5 / 3.0, 1e-6);

        const auto yaw_mid = sampleScript(23.0);  // 偏航中点 45°
        const double yaw = 2.0 * std::atan2(yaw_mid.q[3], yaw_mid.q[0]);
        CHECK_NEAR(yaw, M_PI / 4.0, 1e-5);
        // 四元数必须归一
        const double qn = std::sqrt(yaw_mid.q[0] * yaw_mid.q[0] + yaw_mid.q[3] * yaw_mid.q[3]);
        CHECK_NEAR(qn, 1.0, 1e-6);

        // 阶段序号
        CHECK(scriptPhaseIndex(0.0) == 0);    // 静止
        CHECK(scriptPhaseIndex(4.0) == 1);    // 前移
        CHECK(scriptPhaseIndex(16.0) == 9);   // 上抬
        CHECK(scriptPhaseIndex(26.9) == 14);  // 保持 90°
    }

    // ---- 2) Probe 统计 ----
    {
        ProbeStats st;
        auto make = [](uint64_t t, float x, float y, float z, uint8_t reset, int8_t quality,
                       uint8_t frame, uint8_t child) {
            mavlink_odometry_t o{};
            std::memset(&o, 0, sizeof(o));
            o.time_usec = t;
            o.x = x;
            o.y = y;
            o.z = z;
            o.q[0] = 1.0f;
            o.reset_counter = reset;
            o.quality = quality;
            o.estimator_type = MAV_ESTIMATOR_TYPE_VIO;
            o.frame_id = frame;
            o.child_frame_id = child;
            return o;
        };

        st.add(make(1000000ULL, 0.0f, 0.0f, 0.0f, 0, 90, 20, 12), 0);
        st.add(make(1033333ULL, 0.1f, 0.0f, 0.0f, 0, 90, 20, 12), 0);
        st.add(make(1066666ULL, 0.2f, 0.0f, 0.0f, 1, 50, 20, 12), 0);  // reset 变化
        st.add(make(1000000ULL, 0.2f, 0.0f, 0.0f, 1, 40, 12, 12), 0);  // 时间倒退 + 坐标系错 + 低质量

        CHECK(st.odometry == 4);
        CHECK(st.reset_changes == 1);
        CHECK(st.time_backwards == 1);
        CHECK(st.min_quality == 40);
        CHECK(st.max_quality == 90);
        CHECK(st.bad_quality == 1);   // 只有 40 低于 VISO_QUAL_MIN=50 的典型门限
        CHECK(st.bad_frame == 1);
        CHECK_NEAR(st.rateHz(), 30.0, 0.5);  // 333333 us 间隔 ≈ 30 Hz
        st.countGaps();
        CHECK(st.gaps == 0);          // 倒退那帧不计入间隔，其余等间隔

        ProbeStats st2;
        mavlink_odometry_t bad_q = make(2000000ULL, 0, 0, 0, 0, 90, 20, 12);
        bad_q.q[0] = 0.5f;            // 非归一四元数
        st2.add(bad_q, 0);
        CHECK(st2.bad_quat == 1);
        std::printf("[2] Probe 统计 OK（帧数/质量/reset/坐标系/乱序/频率）\n");
    }

    if (g_failed == 0) {
        std::printf("PASS: 合成轨迹与 ODOMETRY 统计逻辑全部正确\n");
        return 0;
    }
    std::printf("FAILED: %d 处不通过\n", g_failed);
    return 1;
}
