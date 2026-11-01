#pragma once

#include "guidance_types.hpp"

// 制导主入口。
// 设计原则：纯函数——只吃 state，只吐 cmd；不读全局、不碰 I/O。
// 仿真阶段用真值喂它，接真机时用视觉喂它，本文件一行不改。

namespace pure_uav {

// 一次制导周期。
// 内部流程：有效性检查 → 几何量 → 选导引律 → 末端冻结 → 过载限幅。
// memory 由调用方持有并跨周期传回（制导本身无全局状态）。
PureGuidanceCmd guidance_step(const GuidanceState &state,
                          const GuidanceParams &params,
                          GuidanceMemory &memory);

} // namespace pure_uav
