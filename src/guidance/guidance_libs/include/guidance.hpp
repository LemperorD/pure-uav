#pragma once

#include "guidance_types.hpp"

// 制导主入口。
// 设计原则：纯函数——只吃 state，只吐 cmd；不读全局、不碰 I/O。
// 仿真阶段用真值喂它，接真机时用视觉喂它，本文件一行不改。

namespace pure_uav {

/// 一次制导周期（纯函数）。
/// @param state 外部口径已转成内部 ENU 的输入。
/// @param params 制导参数。
/// @param memory 跨周期记忆，由调用方持有并传回。
/// @return 本周期指令（含 t_go/Vc/R/lam_dot 等调试量）。
PureGuidanceCmd guidance_step(const GuidanceState &state,
                          const GuidanceParams &params,
                          GuidanceMemory &memory);

} // namespace pure_uav
