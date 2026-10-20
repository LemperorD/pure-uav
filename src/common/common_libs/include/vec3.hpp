#pragma once

#include <cmath>

// 轻量三维向量。
// 制导自带，不依赖外部线性代数库（Eigen 等），方便将来原样搬上机载。
// 约定：所有量均为 double；矢量默认在惯性系（建议 ENU）。

namespace pure_uav {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3 &operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};

inline Vec3 operator*(double s, const Vec3 &v) { return v * s; }

inline double dot(const Vec3 &a, const Vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 叉积：3D 制导求法向的核心运算
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline double norm(const Vec3 &v) { return std::sqrt(dot(v, v)); }

// 单位化；零矢量返回零矢量（避免除零）
inline Vec3 normalized(const Vec3 &v) {
    const double n = norm(v);
    return n > 1e-12 ? v / n : Vec3{};
}

} // namespace pure_uav
