// VisionPoseConvert 的纯数学单元测试（不依赖 OpenVINS / 相机 / 飞控）。
//
// 约定（与 OpenVINS 一致）：G 系 z 轴向上，xy 为水平面；
// LOCAL_FRD 的 z 指向重力（G 的 -z），x 取初始化时机头在水平面的投影。

#include <cmath>
#include <cstdio>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pure_uav/vision_pose_convert.hpp"

namespace {

int g_failures = 0;

void expectNear(double got, double want, double tol, const char* what) {
    if (std::fabs(got - want) > tol) {
        std::fprintf(stderr, "FAIL %-40s got=%.9f want=%.9f\n", what, got, want);
        g_failures++;
    }
}

void expectVec(const Eigen::Vector3d& got, const Eigen::Vector3d& want, double tol,
               const char* what) {
    if ((got - want).norm() > tol) {
        std::fprintf(stderr, "FAIL %-40s got=(%.6f %.6f %.6f) want=(%.6f %.6f %.6f)\n", what,
                     got.x(), got.y(), got.z(), want.x(), want.y(), want.z());
        g_failures++;
    }
}

void expectQuatIdentity(const pure_uav::VisionPose& p, const char* what) {
    const double s = (p.q[0] >= 0.0f) ? 1.0 : -1.0;
    expectNear(s * p.q[0], 1.0, 1e-5, what);
    expectNear(s * p.q[1], 0.0, 1e-5, what);
    expectNear(s * p.q[2], 0.0, 1e-5, what);
    expectNear(s * p.q[3], 0.0, 1e-5, what);
}

Eigen::Matrix3d rotX(double a) {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix();
}
Eigen::Matrix3d rotZ(double a) {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

// 机体水平（FRD：x前 y右 z下）放在 z-up 的 G 系里，姿态为绕 x 转 180°。
Eigen::Matrix3d levelBodyToG() { return rotX(M_PI); }

// 场景 1：水平、原地不动
void testStaticLevel() {
    const Eigen::Matrix3d R_BtoG0 = levelBodyToG();
    pure_uav::VisionPoseConvert conv;
    conv.reset(R_BtoG0.transpose(), Eigen::Vector3d::Zero());

    auto p = conv.convert(100.0, R_BtoG0.transpose(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());
    expectVec(Eigen::Vector3d(p.x, p.y, p.z), Eigen::Vector3d::Zero(), 1e-6, "static p");
    expectQuatIdentity(p, "static q");
    expectNear(p.vx, 0.0, 1e-6, "static vx");
    if (p.timestamp_us != 100000000ULL) {
        std::fprintf(stderr, "FAIL timestamp_us got=%llu\n",
                     static_cast<unsigned long long>(p.timestamp_us));
        g_failures++;
    }
    if (p.frame_id != pure_uav::VisionFrame::LOCAL_FRD ||
        p.child_frame_id != pure_uav::VisionFrame::BODY_FRD) {
        std::fprintf(stderr, "FAIL frame ids\n");
        g_failures++;
    }
}

// 场景 2：初始化后机头绕世界 z（向上）转 +90°，即机头指向世界 +y；
// 沿机头方向移动 1 m、前飞 1 m/s。
void testYawAndTranslation() {
    const Eigen::Matrix3d R_BtoG0 = levelBodyToG();          // 初始化：机头 = 世界 +x
    const Eigen::Matrix3d R_BtoG = rotZ(M_PI / 2.0) * R_BtoG0;  // 机头 = 世界 +y

    pure_uav::VisionPoseConvert conv;
    conv.reset(R_BtoG0.transpose(), Eigen::Vector3d::Zero());

    // 沿机头（世界 +y）移动 1 m，速度 1 m/s
    const Eigen::Vector3d p_G(0.0, 1.0, 0.0);
    const Eigen::Vector3d v_G(0.0, 1.0, 0.0);

    auto p = conv.convert(10.0, R_BtoG.transpose(), p_G, v_G);
    // LOCAL_FRD：x = 世界 +x，y = -世界 y，z = -世界 z
    expectVec(Eigen::Vector3d(p.x, p.y, p.z), Eigen::Vector3d(0.0, -1.0, 0.0), 1e-6,
              "yaw90 p");
    // 机体系速度：前飞 = (1,0,0)
    expectVec(Eigen::Vector3d(p.vx, p.vy, p.vz), Eigen::Vector3d(1.0, 0.0, 0.0), 1e-6,
              "yaw90 v_body");

    // q：相对初始化机头转了 +90°（世界 z 向上），在机体系里表现为绕 z 轴 -90°
    const double c = std::cos(M_PI / 4.0), s = std::sin(M_PI / 4.0);
    const double sign = (p.q[0] >= 0.0f) ? 1.0 : -1.0;
    expectNear(sign * p.q[0], c, 1e-5, "yaw90 qw");
    expectNear(sign * p.q[3], -s, 1e-5, "yaw90 qz");
}

// 场景 3：初始化时已有 20° 横滚；LOCAL_FRD 的 z 仍严格指向重力
void testTiltedInit() {
    const double roll = 20.0 * M_PI / 180.0;
    const Eigen::Matrix3d R_BtoG0 = rotX(M_PI + roll);  // 水平姿态再绕机体 x 滚 20°

    pure_uav::VisionPoseConvert conv;
    conv.reset(R_BtoG0.transpose(), Eigen::Vector3d::Zero());

    auto p0 = conv.convert(1.0, R_BtoG0.transpose(), Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero());
    const double sign = (p0.q[0] >= 0.0f) ? 1.0 : -1.0;
    expectNear(sign * p0.q[0], std::cos(roll / 2.0), 1e-5, "tilt init qw");
    expectNear(sign * p0.q[1], std::sin(roll / 2.0), 1e-5, "tilt init qx");
    expectVec(Eigen::Vector3d(p0.x, p0.y, p0.z), Eigen::Vector3d::Zero(), 1e-6,
              "tilt init p");

    // 沿世界 +x 移动 1 m：LOCAL_FRD 里是 (1,0,0)
    auto p1 = conv.convert(2.0, R_BtoG0.transpose(), Eigen::Vector3d(1.0, 0.0, 0.0),
                           Eigen::Vector3d::Zero());
    expectVec(Eigen::Vector3d(p1.x, p1.y, p1.z), Eigen::Vector3d(1.0, 0.0, 0.0), 1e-6,
              "tilt move p");
}

// 场景 4：IMU→机体系安装旋转：机体系 x 轴指向 IMU 的 +y
void testBodyFromImuRotation() {
    const Eigen::Matrix3d R_ItoB = rotZ(-M_PI / 2.0);
    pure_uav::PoseConvertOptions opt;
    opt.R_imu_to_body = R_ItoB;

    const Eigen::Matrix3d R_GtoI0 = levelBodyToG().transpose();  // IMU 水平

    pure_uav::VisionPoseConvert conv(opt);
    conv.reset(R_GtoI0, Eigen::Vector3d::Zero());

    auto p = conv.convert(1.0, R_GtoI0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    expectQuatIdentity(p, "body_from_imu init q");

    // 沿机头方向（机体系 x，即 IMU +y）移动 1 m、速度 1 m/s
    // 世界系下 IMU +y = -世界 y
    const Eigen::Vector3d p_G(0.0, -1.0, 0.0);
    const Eigen::Vector3d v_G(0.0, -1.0, 0.0);
    auto p2 = conv.convert(2.0, R_GtoI0, p_G, v_G);
    expectVec(Eigen::Vector3d(p2.vx, p2.vy, p2.vz), Eigen::Vector3d(1.0, 0.0, 0.0), 1e-6,
              "body_from_imu v_body");
    expectVec(Eigen::Vector3d(p2.x, p2.y, p2.z), Eigen::Vector3d(1.0, 0.0, 0.0), 1e-6,
              "body_from_imu p");
}

// 场景 5：杆臂：机体系原点在 IMU 前方 0.1 m，IMU 绕 z 旋转时产生速度
void testLeverArm() {
    pure_uav::PoseConvertOptions opt;
    opt.t_imu_body = Eigen::Vector3d(0.1, 0.0, 0.0);

    const Eigen::Matrix3d R_GtoI = levelBodyToG().transpose();
    pure_uav::VisionPoseConvert conv(opt);
    conv.reset(R_GtoI, Eigen::Vector3d::Zero());

    // IMU 静止但绕 z 转 1 rad/s：机体系原点速度 = ω × r = (0, 0.1, 0)
    auto p = conv.convert(1.0, R_GtoI, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d(0.0, 0.0, 1.0));
    expectVec(Eigen::Vector3d(p.x, p.y, p.z), Eigen::Vector3d::Zero(), 1e-6, "lever p0");
    expectVec(Eigen::Vector3d(p.vx, p.vy, p.vz), Eigen::Vector3d(0.0, 0.1, 0.0), 1e-6,
              "lever v");
}

}  // namespace

int main() {
    testStaticLevel();
    testYawAndTranslation();
    testTiltedInit();
    testBodyFromImuRotation();
    testLeverArm();

    if (g_failures == 0) {
        std::printf("PASS: VisionPoseConvert 全部用例通过\n");
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d 个检查未通过\n", g_failures);
    return 1;
}
