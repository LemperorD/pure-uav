#pragma once

#include "shm_camera_source.hpp"

namespace pure_uav::app {
// 保留组合根的类型名；实际输入来自外部 camera_server。
using CameraService = ShmCameraSource;
}  // namespace pure_uav::app
