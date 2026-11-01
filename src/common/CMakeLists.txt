# ============================================================
#  公共基础件（common）—— 零业务依赖，全项目可复用
#
#    pure_math_tools : 标量数学（deg2rad / rad2deg / 激活函数…）
#    pure_vec3       : 三维向量 Vec3（**纯头文件**，INTERFACE 目标，不产生 .so）
#    mavlink_units   : MAVLink 单位换算 / 哨兵 / uint32 回绕
#
#  ★ 2026-10-20 目录调整：原 `src/interface/` 撤销 ——
#      · mavlink_units → 本目录（纯数值换算，零依赖，谁都能用）
#      · vec3.hpp     → 本目录（纯几何类型，原本在 guidance_libs/include）
#      · state_adapter / state_check → `src/guidance/adapter/`
#        （它们产出 GuidanceState，是制导的输入边界，不属于通用件）
#
#  依赖方向：common 不依赖任何上层模块（guidance / driver / app）。
#  三者共用同一个 include 目录，因此源码里的 include 写法保持不变：
#    #include "vec3.hpp" / "mavlink_units.hpp" / "pure_math_tools.hpp"
# ============================================================

add_library(pure_math_tools SHARED
    common_libs/src/pure_math_tools.cpp
)
target_include_directories(pure_math_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)

# ------------------------------------------------------------
#  pure_vec3：三维向量（header-only）
#
#  为什么是 INTERFACE 而不是 SHARED：
#    vec3.hpp 全是 inline 函数 / 结构体，没有 .cpp，编译不出库文件；
#    做成 INTERFACE 后，任何 `target_link_libraries(x pure_vec3)` 只继承 include
#    目录、**不引入运行时依赖** —— 这正是"机载少带一个 .so"想要的效果。
#  使用方：guidance_core / guidance_sim / state_adapter / test_state_adapter。
# ------------------------------------------------------------
add_library(pure_vec3 INTERFACE)
target_include_directories(pure_vec3 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)

# ------------------------------------------------------------
#  mavlink_units：MAVLink 单位换算（纯函数、零依赖）
#    · 不解析报文、不做坐标变换（坐标 / 姿态 / 时间对齐在 src/guidance/adapter）
#    · 只用 <cmath> / <cstdint>，因此解析侧、制导侧、仿真侧都可直接链接
# ------------------------------------------------------------
add_library(mavlink_units SHARED
    common_libs/src/mavlink_units.cpp
)
target_include_directories(mavlink_units PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)

install(TARGETS pure_math_tools mavlink_units
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
