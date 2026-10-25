#include "pure_uav/pose_logger.hpp"

#include <cstdio>
#include <filesystem>
#include <iomanip>

namespace pure_uav {

namespace {
void ensureParentDir(const std::string& path) {
    if (path.empty()) return;
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
    }
}
}  // namespace

PoseLogger::~PoseLogger() { close(); }

bool PoseLogger::open(const std::string& csv_path, const std::string& tum_path) {
    close();
    bool ok = true;

    if (!csv_path.empty()) {
        ensureParentDir(csv_path);
        csv_.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            std::fprintf(stderr, "[PoseLogger] 无法打开 CSV: %s\n", csv_path.c_str());
            ok = false;
        } else {
            csv_ << "timestamp_us,frame_id,child_frame_id,"
                    "x,y,z,qw,qx,qy,qz,vx,vy,vz,cam_imu_dt\n";
        }
    }

    if (!tum_path.empty()) {
        ensureParentDir(tum_path);
        tum_.open(tum_path, std::ios::out | std::ios::trunc);
        if (!tum_.is_open()) {
            std::fprintf(stderr, "[PoseLogger] 无法打开 TUM: %s\n", tum_path.c_str());
            ok = false;
        } else {
            tum_ << "# timestamp tx ty tz qx qy qz qw\n";
        }
    }

    return ok;
}

void PoseLogger::log(const VisionPose& pose) {
    if (csv_.is_open()) {
#ifdef PURE_UAV_VISIONPOSE_NO_CAM_IMU_DT
        const float cam_imu_dt_out = -999.0f;   // 仓库版 VisionPose 无此字段
#else
        const float cam_imu_dt_out = pose.cam_imu_dt;
#endif
        csv_ << pose.timestamp_us << ','
             << static_cast<int>(pose.frame_id) << ','
             << static_cast<int>(pose.child_frame_id) << ','
             << std::setprecision(9)
             << pose.x << ',' << pose.y << ',' << pose.z << ','
             << pose.q[0] << ',' << pose.q[1] << ',' << pose.q[2] << ',' << pose.q[3] << ','
             << pose.vx << ',' << pose.vy << ',' << pose.vz << ',' << cam_imu_dt_out << '\n';
        csv_.flush();
    }
    if (tum_.is_open()) {
        const double ts = static_cast<double>(pose.timestamp_us) * 1e-6;
        tum_ << std::fixed << std::setprecision(6) << ts << ' '
             << std::setprecision(6)
             << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
             << pose.q[1] << ' ' << pose.q[2] << ' ' << pose.q[3] << ' ' << pose.q[0] << '\n';
        // 每行刷新：rviz 的实时桥接要能“边跑边读”，ofstream 默认 8 KB 缓冲会晚 ~8 s
        tum_.flush();
    }
}

void PoseLogger::close() {
    if (csv_.is_open()) csv_.close();
    if (tum_.is_open()) tum_.close();
}

}  // namespace pure_uav
