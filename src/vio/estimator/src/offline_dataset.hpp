// 离线数据集读取：把 ROS bag 转成 images/ + images.csv + imu.csv 后回放。
//
// 目录格式（tools/bag_to_dataset.py 生成）：
//   dataset/
//     images.csv : timestamp_us,filename
//     imu.csv    : timestamp_us,gx,gy,gz,ax,ay,az
//     images/...
//
// 时间戳单位微秒，保持 bag header.stamp 原值（session05 里是 Unix 微秒）。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pure_uav/vio_types.hpp"

namespace pure_uav {

struct DatasetImage {
    uint64_t timestamp_us = 0;
    std::string path;  // 绝对路径
};

class OfflineDataset {
public:
    bool load(const std::string& dataset_dir, std::string* error);

    const std::vector<DatasetImage>& images() const { return images_; }
    const std::vector<ImuSample>& imu() const { return imu_; }

    const std::string& dir() const { return dir_; }

private:
    std::string dir_;
    std::vector<DatasetImage> images_;
    std::vector<ImuSample> imu_;
};

}  // namespace pure_uav
