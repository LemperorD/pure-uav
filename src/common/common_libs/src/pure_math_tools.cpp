#include "pure_math_tools.hpp"

inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

inline double ReLU(double x) {
    return x > 0 ? x : 0;
}

inline double SiLU(double x) {
    return x / (1 + exp(-x));
}

inline double leaky_ReLU(double x) {
    return x > 0 ? x : 0.01 * x;
}

inline double sigmoid(double x) {
    return 1 / (1 + exp(-x));
}