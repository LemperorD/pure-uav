// ODOMETRY 打包回归测试：不需要飞控、不需要串口、不需要相机。
//
// 目的：把「打出来的字节」重新解析回来，逐字段核对。只看 sendVisionPose() 的
// 返回值是发现不了字段顺序错误的 —— 那正是 2026-10-28 之前的状态：
//   mavlink_msg_odometry_pack() 的最后三个参数是
//       (reset_counter, estimator_type, quality)
//   旧调用按 (quality, estimator_type, reset_counter) 传，于是
//       "质量 90 / 重置 0" 实际发成 "重置 90 / 质量 0"。
//
// 这个测试同时检查两件事：
//   1) 语义：mavlink 官方 decode 出来的 reset_counter / estimator_type / quality 对不对；
//   2) 线上字节：payload 最后三个字节（reset/estimator/quality）独立核对，
//      防止"编码和解码用同一份错误 header，于是对称地错、测试照样通过"。
//
// 运行：ctest -R odometry_pack --output-on-failure
//   或直接 ./bin/test_odometry_pack

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "common/mavlink.h"

#include "mavlink_link.hpp"

namespace {

int g_failed = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  ❌ 第 %d 行不成立: %s\n", __LINE__, #cond);        \
            ++g_failed;                                                      \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        const double _va = static_cast<double>(a);                            \
        const double _vb = static_cast<double>(b);                            \
        if (std::fabs(_va - _vb) > (tol)) {                                   \
            std::printf("  ❌ 第 %d 行: %s=%.9g 与 %s=%.9g 相差超过 %.3g\n",  \
                        __LINE__, #a, _va, #b, _vb, (double)(tol));           \
            ++g_failed;                                                      \
        }                                                                    \
    } while (0)

// 把发送缓冲解析回 mavlink_message_t；返回 false 表示帧头/CRC/长度有问题。
bool parseFrame(const uint8_t* buf, uint16_t len, mavlink_message_t& msg) {
    mavlink_status_t status{};
    for (uint16_t i = 0; i < len; ++i) {
        // 用独立 channel，避免和 ABI 里的 COMM_0 状态串味（本测试里并不启动链路）
        if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status)) return true;
    }
    return false;
}

struct WireFields {
    uint8_t reset_counter = 0;
    uint8_t estimator_type = 0;
    int8_t quality = 0;
    uint16_t payload_off = 0;
    uint16_t payload_len = 0;
};

// 直接从原始字节里抠出最后三个字段（ODOMETRY payload 布局的末尾就是它们）。
// 不依赖 mavlink_msg_odometry_decode，专门用来交叉验证。
void extractTailFields(const uint8_t* buf, uint16_t len, uint16_t payload_len, WireFields& out) {
    out.payload_len = payload_len;
    // 帧总长 = 帧头 + payload + 2 字节 CRC（未签名）。v1 头 6 字节，v2 头 10 字节。
    const uint16_t overhead = static_cast<uint16_t>(len - payload_len - 2);
    out.payload_off = overhead;
    if (payload_len < 3 || overhead > len) return;
    out.reset_counter = buf[overhead + payload_len - 3];
    out.estimator_type = buf[overhead + payload_len - 2];
    out.quality = static_cast<int8_t>(buf[overhead + payload_len - 1]);
}

// 用例 1：正常健康状态（质量 90 / 重置 3 / 有协方差）
void caseNormal() {
    std::printf("[用例1] OK 状态：quality=90 reset=3\n");
    pure_uav::VisionPose p;
    p.timestamp_us = 1787654321ULL;  // 任意飞控 boot us
    p.frame_id = pure_uav::VisionFrame::LOCAL_FRD;
    p.child_frame_id = pure_uav::VisionFrame::BODY_FRD;
    p.x = 1.25f;
    p.y = -0.5f;
    p.z = 2.75f;
    p.q[0] = 0.9914449f;  // yaw≈15°
    p.q[1] = 0.0f;
    p.q[2] = 0.0f;
    p.q[3] = 0.1305262f;
    p.vx = 0.3f;
    p.vy = -0.2f;
    p.vz = 0.05f;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(
        p, /*quality=*/90, /*reset_counter=*/3, /*pos_sigma=*/0.15f, /*vel_sigma=*/0.25f,
        /*sysid=*/2, /*compid=*/1, buf, sizeof(buf));
    CHECK(len > 0);
    CHECK(len <= sizeof(buf));

    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));  // CRC 错会在这里失败
    CHECK(msg.msgid == MAVLINK_MSG_ID_ODOMETRY);

    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);

    // ---- 语义层 ----
    CHECK(odo.time_usec == p.timestamp_us);
    CHECK(odo.frame_id == 20);        // MAV_FRAME_LOCAL_FRD
    CHECK(odo.child_frame_id == 12);  // MAV_FRAME_BODY_FRD
    CHECK_NEAR(odo.x, 1.25, 1e-6);
    CHECK_NEAR(odo.y, -0.5, 1e-6);
    CHECK_NEAR(odo.z, 2.75, 1e-6);
    CHECK_NEAR(odo.q[0], 0.9914449, 1e-6);
    CHECK_NEAR(odo.q[3], 0.1305262, 1e-6);
    CHECK_NEAR(odo.vx, 0.3, 1e-6);
    CHECK_NEAR(odo.vy, -0.2, 1e-6);
    CHECK_NEAR(odo.vz, 0.05, 1e-6);
    // ★ 这三行就是这个 bug 的回归点
    CHECK(odo.reset_counter == 3);
    CHECK(odo.estimator_type == MAV_ESTIMATOR_TYPE_VIO);
    CHECK(odo.quality == 90);

    // ---- 线上字节层（独立于 decode）----
    WireFields w;
    extractTailFields(buf, len, msg.len, w);
    CHECK(w.payload_off == 6 || w.payload_off == 10);
    CHECK(w.reset_counter == 3);
    CHECK(w.estimator_type == MAV_ESTIMATOR_TYPE_VIO);
    CHECK(w.quality == 90);

    // ---- 协方差：位置/速度对角线 = sigma² ----
    CHECK_NEAR(odo.pose_covariance[0], 0.15 * 0.15, 1e-7);
    CHECK_NEAR(odo.pose_covariance[6], 0.15 * 0.15, 1e-7);
    CHECK_NEAR(odo.pose_covariance[11], 0.15 * 0.15, 1e-7);
    CHECK_NEAR(odo.velocity_covariance[0], 0.25 * 0.25, 1e-7);
    CHECK_NEAR(odo.velocity_covariance[11], 0.25 * 0.25, 1e-7);
    // 姿态协方差没有估计值，保持 0（飞控侧会 constrain 到配置下限）
    CHECK_NEAR(odo.pose_covariance[15], 0.0, 1e-9);
}

// 用例 2：降级状态。质量从 90 变到 50 时，reset_counter 必须**保持不变**——
// 否则飞控会把每一次健康状态变化都当成"估计器重置"，触发重锚定。
void caseDegradedKeepsReset() {
    std::printf("[用例2] DEGRADED 状态：quality=50，reset 不变\n");
    pure_uav::VisionPose p;
    p.timestamp_us = 1787654322ULL;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    const uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(
        p, 50, 3, 0.5f, 0.8f, 2, 1, buf, sizeof(buf));
    CHECK(len > 0);
    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));
    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(odo.quality == 50);
    CHECK(odo.reset_counter == 3);  // ← 关键：健康状态变化不能改 reset
}

// 用例 3：未知标准差 → 协方差首元素 NaN（= 未知），不能是 0（= 完美已知）
void caseUnknownCovariance() {
    std::printf("[用例3] 未提供 sigma：协方差应为 NaN(未知)\n");
    pure_uav::VisionPose p;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len =
        pure_uav::MavlinkLink::encodeVisionPose(p, 0, 0, 0.0f, 0.0f, 2, 1, buf, sizeof(buf));
    CHECK(len > 0);
    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));
    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(std::isnan(odo.pose_covariance[0]));
    CHECK(std::isnan(odo.pose_covariance[6]));
    CHECK(std::isnan(odo.pose_covariance[11]));
    CHECK(std::isnan(odo.velocity_covariance[0]));
    CHECK(std::isnan(odo.velocity_covariance[6]));
    CHECK(odo.reset_counter == 0);
    CHECK(odo.quality == 0);
}

// 用例 3b：只给位置标准差 / 只给速度标准差 —— 两者必须各自独立判断，
// 不能因为另一个未知就一起填 NaN（2026-10-29 验收指出）。
void caseIndependentSigma() {
    std::printf("[用例3b] 只给位置 sigma：位置协方差有效、速度协方差 NaN\n");
    pure_uav::VisionPose p;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(p, 90, 0, /*pos=*/0.15f,
                                                           /*vel=*/0.0f, 2, 1, buf, sizeof(buf));
    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));
    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK_NEAR(odo.pose_covariance[0], 0.15 * 0.15, 1e-7);
    CHECK_NEAR(odo.pose_covariance[6], 0.15 * 0.15, 1e-7);
    CHECK_NEAR(odo.pose_covariance[11], 0.15 * 0.15, 1e-7);
    CHECK(std::isnan(odo.velocity_covariance[0]));   // 速度未知
    CHECK(std::isnan(odo.velocity_covariance[6]));

    std::printf("[用例3c] 只给速度 sigma：速度协方差有效、位置协方差 NaN\n");
    len = pure_uav::MavlinkLink::encodeVisionPose(p, 90, 0, /*pos=*/0.0f, /*vel=*/0.25f,
                                                 2, 1, buf, sizeof(buf));
    CHECK(parseFrame(buf, len, msg));
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(std::isnan(odo.pose_covariance[0]));       // 位置未知
    CHECK_NEAR(odo.velocity_covariance[0], 0.25 * 0.25, 1e-7);
    CHECK_NEAR(odo.velocity_covariance[11], 0.25 * 0.25, 1e-7);
}

// 用例 4：质量越界要被夹到 [0,100]
void caseQualityClamp() {
    std::printf("[用例4] quality 越界：150→100，-20→0\n");
    pure_uav::VisionPose p;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(p, 150, 0, 0.1f, 0.1f, 2, 1, buf,
                                                           sizeof(buf));
    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));
    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(odo.quality == 100);

    len = pure_uav::MavlinkLink::encodeVisionPose(p, -20, 0, 0.1f, 0.1f, 2, 1, buf, sizeof(buf));
    CHECK(parseFrame(buf, len, msg));
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(odo.quality == 0);
}

// 用例 5：缓冲区不足必须返回 0，不能越界写
void caseSmallBuffer() {
    std::printf("[用例5] 缓冲区不足：返回 0\n");
    pure_uav::VisionPose p;
    uint8_t tiny[8];
    const uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(p, 90, 0, 0.1f, 0.1f, 2, 1, tiny,
                                                                 sizeof(tiny));
    CHECK(len == 0);
}

// 用例 6：reset_counter=255 是 uint8 上限（重建 256 次的边界），不能溢出成别的东西
void caseResetWrapBoundary() {
    std::printf("[用例6] reset=255 边界\n");
    pure_uav::VisionPose p;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = pure_uav::MavlinkLink::encodeVisionPose(p, 90, 255, 0.1f, 0.1f, 2, 1,
                                                                 buf, sizeof(buf));
    mavlink_message_t msg{};
    CHECK(parseFrame(buf, len, msg));
    mavlink_odometry_t odo{};
    mavlink_msg_odometry_decode(&msg, &odo);
    CHECK(odo.reset_counter == 255);
    CHECK(odo.quality == 90);
}

}  // namespace

int main() {
    std::printf("===== ODOMETRY 打包/解析回归测试 =====\n");
    caseNormal();
    caseDegradedKeepsReset();
    caseUnknownCovariance();
    caseIndependentSigma();
    caseQualityClamp();
    caseSmallBuffer();
    caseResetWrapBoundary();

    if (g_failed == 0) {
        std::printf("PASS: ODOMETRY 字段顺序、协方差、质量夹紧全部正确\n");
        return 0;
    }
    std::printf("FAILED: %d 处不通过\n", g_failed);
    return 1;
}
