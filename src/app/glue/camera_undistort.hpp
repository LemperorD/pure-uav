#pragma once

#include "glue/guidance_session.hpp"

namespace pure_uav::app {

/// 把原始彩色像素校正为同一针孔内参下的无畸变像素。
/// @return 相机已标定、像素和校正结果均有效时为 true。
bool undistortPixel(const CameraCalibration& camera, double u_px, double v_px, double& u_out, double& v_out);

}  // namespace pure_uav::app
