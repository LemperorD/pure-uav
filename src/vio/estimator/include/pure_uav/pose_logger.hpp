// 把 VisionPose 写成 CSV / TUM 轨迹，便于离线分析（evo、python）。
//
// CSV: timestamp_us,frame_id,child_frame_id,x,y,z,qw,qx,qy,qz,vx,vy,vz
// TUM: timestamp_s tx ty tz qx qy qz qw

#pragma once

#include <fstream>
#include <string>

#include "pure_uav/vision_pose.hpp"

namespace pure_uav {

class PoseLogger {
public:
    PoseLogger() = default;
    ~PoseLogger();

    PoseLogger(const PoseLogger&) = delete;
    PoseLogger& operator=(const PoseLogger&) = delete;

    // 路径为空表示不写该格式。返回是否成功打开所有请求的文件。
    bool open(const std::string& csv_path, const std::string& tum_path);

    void log(const VisionPose& pose);

    void close();

    bool isOpen() const { return csv_.is_open() || tum_.is_open(); }

private:
    std::ofstream csv_;
    std::ofstream tum_;
};

}  // namespace pure_uav
