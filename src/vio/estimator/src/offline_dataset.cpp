#include "offline_dataset.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

namespace pure_uav {

namespace {

bool isHeader(const std::string& line) {
    if (line.empty()) return true;
    if (line[0] == '#') return true;
    // 第一个字段不是数字就认为是表头
    const char c = line[0];
    return !(std::isdigit(static_cast<unsigned char>(c)) || c == '-');
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // 去空白
        const auto b = item.find_first_not_of(" \t\r\n");
        const auto e = item.find_last_not_of(" \t\r\n");
        out.push_back(b == std::string::npos ? std::string() : item.substr(b, e - b + 1));
    }
    return out;
}

bool toU64(const std::string& s, uint64_t& v) {
    try {
        size_t idx = 0;
        unsigned long long x = std::stoull(s, &idx);
        if (idx != s.size()) return false;
        v = static_cast<uint64_t>(x);
        return true;
    } catch (...) {
        return false;
    }
}

bool toDouble(const std::string& s, double& v) {
    try {
        size_t idx = 0;
        v = std::stod(s, &idx);
        return idx == s.size();
    } catch (...) {
        return false;
    }
}

}  // namespace

bool OfflineDataset::load(const std::string& dataset_dir, std::string* error) {
    dir_ = dataset_dir;
    images_.clear();
    imu_.clear();

    const std::filesystem::path root(dataset_dir);
    const std::filesystem::path images_csv = root / "images.csv";
    const std::filesystem::path imu_csv = root / "imu.csv";

    if (!std::filesystem::exists(images_csv)) {
        if (error) *error = "找不到 " + images_csv.string();
        return false;
    }
    if (!std::filesystem::exists(imu_csv)) {
        if (error) *error = "找不到 " + imu_csv.string();
        return false;
    }

    // images.csv
    {
        std::ifstream f(images_csv);
        if (!f.is_open()) {
            if (error) *error = "无法打开 " + images_csv.string();
            return false;
        }
        std::string line;
        int line_no = 0;
        while (std::getline(f, line)) {
            line_no++;
            if (isHeader(line)) continue;
            const auto cols = splitCsv(line);
            if (cols.size() < 2) continue;
            DatasetImage img;
            if (!toU64(cols[0], img.timestamp_us)) {
                if (error) *error = "images.csv 第 " + std::to_string(line_no) + " 行时间戳非法";
                return false;
            }
            std::filesystem::path p(cols[1]);
            img.path = p.is_absolute() ? p.string() : (root / p).string();
            images_.push_back(std::move(img));
        }
    }

    // imu.csv
    {
        std::ifstream f(imu_csv);
        if (!f.is_open()) {
            if (error) *error = "无法打开 " + imu_csv.string();
            return false;
        }
        std::string line;
        int line_no = 0;
        while (std::getline(f, line)) {
            line_no++;
            if (isHeader(line)) continue;
            const auto cols = splitCsv(line);
            if (cols.size() < 7) continue;
            ImuSample s;
            double gx, gy, gz, ax, ay, az;
            if (!toU64(cols[0], s.timestamp_us) || !toDouble(cols[1], gx) ||
                !toDouble(cols[2], gy) || !toDouble(cols[3], gz) ||
                !toDouble(cols[4], ax) || !toDouble(cols[5], ay) ||
                !toDouble(cols[6], az)) {
                if (error) *error = "imu.csv 第 " + std::to_string(line_no) + " 行格式非法";
                return false;
            }
            s.gx = gx;
            s.gy = gy;
            s.gz = gz;
            s.ax = ax;
            s.ay = ay;
            s.az = az;
            imu_.push_back(s);
        }
    }

    std::sort(images_.begin(), images_.end(),
              [](const DatasetImage& a, const DatasetImage& b) {
                  return a.timestamp_us < b.timestamp_us;
              });
    std::sort(imu_.begin(), imu_.end(), [](const ImuSample& a, const ImuSample& b) {
        return a.timestamp_us < b.timestamp_us;
    });

    if (images_.empty()) {
        if (error) *error = "数据集中没有图像";
        return false;
    }
    if (imu_.empty()) {
        if (error) *error = "数据集中没有 IMU 数据";
        return false;
    }
    return true;
}

}  // namespace pure_uav
