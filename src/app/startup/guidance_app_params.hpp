#pragma once
// =============================================================================
//  guidance_app_params.hpp —— 宿主侧的整机启动配置（app 专用）
//
//  这里只放“main 的宿主应用”需要、而制导库不应该知道的配置。
//  制导库自己的参数是 pure_uav::GuidanceParams；飞行外壳配置是
//  pure_uav::guidance::flight::GuidanceLoopConfig。
// =============================================================================

#include "guidance_flight_types.hpp"
#include "guidance_types.hpp"


namespace pure_uav::app {

// 短别名：宿主侧代码用 flight:: 前缀引用制导飞行外壳的类型。
namespace flight = pure_uav::guidance::flight;

/// 整机启动配置：制导参数 + 飞行外壳配置。
struct GuidanceAppParams {
    GuidanceParams guidance;
    flight::GuidanceLoopConfig loop;
};

} // namespace pure_uav::app

