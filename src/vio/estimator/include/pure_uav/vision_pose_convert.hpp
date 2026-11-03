// OpenVINS 状态 → pure_uav::VisionPose 的坐标/时间转换。
//
// OpenVINS 的 global 系 G：z 轴向上（gravity_mag 在 +z，传播方程里取减号），
// x/y 由初始化决定。
// 本模块不依赖 OpenVINS 的 yaw 约定，而是在第一次拿到已初始化状态时锁存：
//
//   LOCAL_FRD（L）：
//     origin = 锁存时刻机体位置
//     z_L    = G 的 -z（重力方向向下）
//     x_L    = 锁存时机体 x 轴在水平面的投影（机头方向）
//     y_L    = z_L × x_L（右）
//
// 输出：
//   p_BinL = R_GtoL * (p_BinG - p_B0inG)         位置（LOCAL_FRD）
//   q_BtoL                                       姿态（机体系相对 LOCAL_FRD，wxyz）
//   v_BinB = R_GtoB * v_BinG                     速度（BODY_FRD）
//
// 机体系 B 与 IMU 系 I 默认重合；若 IMU 有安装旋转/杆臂，通过
// PoseConvertOptions 指定 R_ItoB 和 t_imu_body（机体系原点在 IMU 系下的坐标）。

#pragma once

#include <Eigen/Dense>

#include "pure_uav/vision_pose.hpp"

namespace pure_uav {

struct PoseConvertOptions {
    // 把 IMU 系向量转到机体系的旋转（R_ItoB）。默认单位阵：机体系 = IMU 系。
    Eigen::Matrix3d R_imu_to_body = Eigen::Matrix3d::Identity();
    // 机体系原点在 IMU 系下的坐标（杆臂）。默认 0。
    Eigen::Vector3d t_imu_body = Eigen::Vector3d::Zero();
};

class VisionPoseConvert {
public:
    explicit VisionPoseConvert(const PoseConvertOptions& options = PoseConvertOptions());

    // 锁存 LOCAL_FRD 的原点/朝向。应在第一次拿到已初始化状态时调用一次。
    //   R_GtoI0 : 初始化时刻 global→IMU 的旋转（OpenVINS 的 q_GtoI 对应矩阵）
    //   p_IinG0 : 初始化时刻 IMU 在 global 系的位置
    void reset(const Eigen::Matrix3d& R_GtoI0, const Eigen::Vector3d& p_IinG0);

    bool initialized() const { return initialized_; }

    // 转换一帧状态。
    //   timestamp_sec : 该状态的时间（秒，与输入 IMU/相机同一时钟域）
    //   R_GtoI        : global→IMU 旋转
    //   p_IinG        : IMU 位置（global）
    //   v_IinG        : IMU 速度（global）
    //   omega_I       : IMU 角速度（IMU 系，已扣零偏），仅用于杆臂速度项
    VisionPose convert(double timestamp_sec, const Eigen::Matrix3d& R_GtoI,
                       const Eigen::Vector3d& p_IinG, const Eigen::Vector3d& v_IinG,
                       const Eigen::Vector3d& omega_I = Eigen::Vector3d::Zero()) const;

    // 调试/测试用
    const Eigen::Matrix3d& R_LtoG() const { return R_LtoG_; }
    const Eigen::Vector3d& p_B0inG() const { return p_B0inG_; }
    const PoseConvertOptions& options() const { return options_; }

private:
    PoseConvertOptions options_;
    bool initialized_ = false;
    Eigen::Matrix3d R_LtoG_ = Eigen::Matrix3d::Identity();  // LOCAL_FRD 各轴在 G 下的方向
    Eigen::Vector3d p_B0inG_ = Eigen::Vector3d::Zero();     // 机体系原点在 G 下的锁存位置

    // 四元数符号连续性（q 与 -q 等价，但连续输出更利于日志/下游）
    mutable Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();
    mutable bool has_last_q_ = false;
};

}  // namespace pure_uav
