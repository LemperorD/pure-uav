# ============================================================
#  制导模块（王乙芃）
#
#  ★ 核心 / 仿真拆成两个 target —— 保证机上不带仿真代码
#
#    guidance_core : 制导算法核心（真机 + 仿真**共用**，零仿真依赖）
#                      · guidance.cpp  制导主流程（纯函数 guidance_step）
#                      · pn_law.cpp    PPN / TPN / APN + ω / Vc
#                      · quat.cpp      四元数通用数学（见 quat.hpp 的说明）
#                    ⇒ 真机主循环用它（机载整体入口见下方 guidance_flight）
#
#    guidance_sim  : 仿真专用（**不上机载**）
#                      · sensor.cpp    "虚拟视觉"（相机投影 / 噪声 / 延迟 / 丢帧）
#                      · target.cpp    弹丸动力学模型（DART / 弹道 / 圆周 …）
#                    ⇒ 依赖 guidance_core（单向，无反向依赖）
#
#    guidance_libs : **向后兼容的聚合名**（= core + sim），仅供既有代码/脚本沿用。
#                    ⚠ 机载**不要**链接它 —— 那会把 Sensor（含 <random> 噪声发生器）
#                      和 TargetModel（弹丸动力学）装进飞机。
#
#    state_adapter : 适配层（外部口径 → GuidanceState），2026-10-20 从 src/interface 迁入
#    guidance_flight : **机载聚合名** = guidance_core + state_adapter + mavlink_units
#                    ⇒ 飞机上只 link 这一个名字（不含任何仿真代码）
#
#    依赖方向（2026-10-20 复核，无反向依赖）：
#        common: vec3.hpp（pure_vec3，header-only）
#        guidance_types.hpp ─ guidance.hpp ─ guidance.cpp   ← core
#        pn_law.hpp ─ pn_law.cpp                            ← core
#        quat.hpp ─ quat.cpp                                ← core
#        sensor.hpp/cpp ─→ core                             ← sim（单向）
#        target.hpp/cpp ─→ vec3.hpp                         ← sim
#        adapter/state_adapter ─→ 纯头文件类型 + mavlink_units（**不** link core）
# ============================================================

add_library(guidance_core SHARED
    guidance_libs/src/guidance.cpp
    guidance_libs/src/pn_law.cpp
    guidance_libs/src/quat.cpp
)
target_include_directories(guidance_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/guidance_libs/include
)

# 说明：2026-10-20 起 `vec3.hpp` 已迁到 `src/common/common_libs/include`，
# 由 INTERFACE 目标 `pure_vec3` 提供 include 目录（header-only，不产生 .so）。
# 仍**不**链接 pure_math_tools —— 它没有向量/矩阵运算，与本核心无关。
# ⚠ 依赖变化：核心现在依赖 common 的**头文件**（不是库文件），
#   因此"制导模块可脱离仓库单独拷走"的说法不再成立。
target_link_libraries(guidance_core PUBLIC
    pure_vec3
)

add_library(guidance_sim SHARED
    guidance_libs/src/sensor.cpp
    guidance_libs/src/target.cpp
)
target_include_directories(guidance_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/guidance_libs/include
)
target_link_libraries(guidance_sim
    guidance_core
)

# 向后兼容的聚合名（老 CMakeLists / 脚本里写 guidance_libs 的地方不用改）。
# ⚠ 它是 INTERFACE 目标 —— 本身不产生库文件，只是"把 core + sim 一起带出来"。
add_library(guidance_libs INTERFACE)
target_link_libraries(guidance_libs INTERFACE
    guidance_core
    guidance_sim
)

# ============================================================
#  适配层（adapter）：外部口径 → 内部口径的**唯一**换算点
#  （2026-10-20 由原 `src/interface/state_adapter` 迁入 —— 队伍不再保留
#    interface 目录；它的输出就是 GuidanceState，本质是"制导模块的输入边界"。）
#
#  ★ 为什么 target 保持独立、**不**并进 guidance_core：
#    ① `state_check` 用 <string> / <cstdio>（体检报告），不该进纯算法核心；
#    ② 保留"机上不要适配层"的裁剪开关；
#    ③ 它**不 link guidance_core** —— 只借用纯头文件的类型（GuidanceState / Vec3），
#       不调用任何核心函数，继续保持原设计的"零链接依赖"（机载可单独复用）。
#  机载想"只记一个名字"时用下面的 guidance_flight 聚合目标。
# ============================================================
add_library(state_adapter SHARED
    adapter/src/state_adapter.cpp
    adapter/src/state_check.cpp
)
target_include_directories(state_adapter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/adapter/include
    ${CMAKE_CURRENT_SOURCE_DIR}/guidance_libs/include
)
target_link_libraries(state_adapter
    pure_vec3
    mavlink_units
)

install(TARGETS state_adapter
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ------------------------------------------------------------
#  guidance_flight：**机载最小足迹**的聚合名（INTERFACE，不产生库文件）
#    = guidance_core（制导算法）+ state_adapter（数据入口）+ mavlink_units（单位换算）
#    飞机上只需 `target_link_libraries(xxx guidance_flight)` 一行。
#  ⚠ 不含 guidance_sim（仿真专用：<random> 噪声发生器 + 弹丸动力学）。
# ------------------------------------------------------------
add_library(guidance_flight INTERFACE)
target_link_libraries(guidance_flight INTERFACE
    guidance_core
    state_adapter
    mavlink_units
)

# 数值仿真可执行文件（离线验证用，**不进机载**）
add_executable(pn_sim
    sim/pn_sim.cpp
)
target_link_libraries(pn_sim
    guidance_sim
)
