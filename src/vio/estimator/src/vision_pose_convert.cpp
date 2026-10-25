#include "pure_uav/vision_pose_convert.hpp"

#include <algorithm>
#include <cmath>

namespace pure_uav {

namespace {

// 把向量投影到水平面（G 系 xy 平面为水平面）并归一化。
// 如果投影长度太小（机头几乎垂直，正常飞行不会出现），退化为 G 的 x 轴。
Eigen::Vector3d horizontalUnit(const Eigen::Vector3d& v) {
    Eigen::Vector3d h(v.x(), v.y(), 0.0);
    const double n = h.norm();
    if (n < 1e-6) {
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    return h / n;
}

}  // namespace

VisionPoseConvert::VisionPoseConvert(const PoseConvertOptions& options) : options_(options) {}

void VisionPoseConvert::reset(const Eigen::Matrix3d& R_GtoI0, const Eigen::Vector3d& p_IinG0) {
    const Eigen::Matrix3d R_ItoG0 = R_GtoI0.transpose();
    // R_BtoG = R_ItoG * R_BtoI = R_ItoG * R_ItoB^T
    const Eigen::Matrix3d R_BtoG0 = R_ItoG0 * options_.R_imu_to_body.transpose();

    // OpenVINS 的 G 系 z 轴向上（gravity_mag 在 +z，传播里是减号）。
    // LOCAL_FRD 的 z 轴必须指向重力方向，即 G 的 -z。
    // x 取机头在水平面的投影，y = z × x（右手 FRD：x前 × y右 = z下）。
    const Eigen::Vector3d z_L = Eigen::Vector3d(0.0, 0.0, -1.0);
    const Eigen::Vector3d x_L = horizontalUnit(R_BtoG0.col(0));
    Eigen::Vector3d y_L = z_L.cross(x_L);
    y_L.normalize();

    R_LtoG_.col(0) = x_L;
    R_LtoG_.col(1) = y_L;
    R_LtoG_.col(2) = z_L;

    p_B0inG_ = p_IinG0 + R_ItoG0 * options_.t_imu_body;

    initialized_ = true;
    has_last_q_ = false;
}

VisionPose VisionPoseConvert::convert(double timestamp_sec, const Eigen::Matrix3d& R_GtoI,
                                      const Eigen::Vector3d& p_IinG,
                                      const Eigen::Vector3d& v_IinG,
                                      const Eigen::Vector3d& omega_I) const {
    VisionPose pose;
    pose.timestamp_us = static_cast<uint64_t>(std::llround(timestamp_sec * 1e6));
    pose.frame_id = VisionFrame::LOCAL_FRD;
    pose.child_frame_id = VisionFrame::BODY_FRD;

    if (!initialized_) {
        return pose;  // 未锁存时给单位位姿（调用方应只在 initialized() 后使用）
    }

    const Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
    // R_BtoG = R_ItoG * R_BtoI，R_BtoI = R_ItoB^T
    const Eigen::Matrix3d R_BtoG = R_ItoG * options_.R_imu_to_body.transpose();
    const Eigen::Matrix3d R_GtoB = R_BtoG.transpose();
    const Eigen::Matrix3d R_GtoL = R_LtoG_.transpose();
    const Eigen::Matrix3d R_BtoL = R_GtoL * R_BtoG;

    const Eigen::Vector3d p_BinG = p_IinG + R_ItoG * options_.t_imu_body;
    const Eigen::Vector3d v_BinG = v_IinG + R_ItoG * (omega_I.cross(options_.t_imu_body));

    const Eigen::Vector3d p_BinL = R_GtoL * (p_BinG - p_B0inG_);
    const Eigen::Vector3d v_BinB = R_GtoB * v_BinG;

    Eigen::Quaterniond q(R_BtoL);
    q.normalize();
    if (has_last_q_ && q.dot(last_q_) < 0.0) {
        q.coeffs() = -q.coeffs();  // 保持符号连续（q 与 -q 等价）
    }
    last_q_ = q;
    has_last_q_ = true;

    pose.x = static_cast<float>(p_BinL.x());
    pose.y = static_cast<float>(p_BinL.y());
    pose.z = static_cast<float>(p_BinL.z());
    pose.q[0] = static_cast<float>(q.w());
    pose.q[1] = static_cast<float>(q.x());
    pose.q[2] = static_cast<float>(q.y());
    pose.q[3] = static_cast<float>(q.z());
    pose.vx = static_cast<float>(v_BinB.x());
    pose.vy = static_cast<float>(v_BinB.y());
    pose.vz = static_cast<float>(v_BinB.z());
    return pose;
}

}  // namespace pure_uav
