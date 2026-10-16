# ============================================================
#  接口适配层（解析侧）—— 王乙芃
#  - mavlink_units : MAVLink 单位换算（纯函数、零依赖）
#  - state_adapter : 外部口径 → 内部口径的**唯一**换算点（坐标/姿态/时间/自检）
#
#  职责分工：
#    mavlink_units  只做"数值 + 单位"的换算 —— 不解析报文、不做坐标变换
#    state_adapter  只做"坐标 / 姿态 / 时间对齐" —— 单位换算一律转调 mavlink_units
#  独立成库的理由：
#    ① 每个换算只有一处实现，改一次全对；
#    ② 取整 / 饱和 / 回绕 / 哨兵这些边界情况可以集中到一处并单独测；
#    ③ 解析代码退化成"读字段 → 调函数"，可读性大幅提升；
#    ④ ★ "边界只转一次"：所有外部口径差异都收敛在 state_adapter，
#       制导核内部（GuidanceState 之后）全程不再做任何换算。
# ============================================================

add_library(mavlink_units SHARED
    mavlink_units/src/mavlink_units.cpp
)
target_include_directories(mavlink_units PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink_units/include
)

# 说明：本库**不依赖任何其它模块**（只用 <cmath> / <cstdint>），
# 因此解析侧、制导侧、仿真侧都可以直接链接，不引入额外耦合。

install(TARGETS mavlink_units
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ------------------------------------------------------------
#  state_adapter：数据对齐适配层
#
#  依赖：
#    · mavlink_units          —— 单位换算 / 回绕 / 哨兵
#    · src/guidance/guidance_libs/include 里的**纯头文件**（vec3.hpp、
#      guidance_types.hpp）—— 只借"内部口径的数据结构"，不链接 guidance_libs。
#      ⇒ 因此本层可脱离制导库单独编译/测试（机载侧单独用也方便）。
#    · 四元数旋转自带一份实现（见 .cpp 注释），同样是为了零链接依赖；
#      两处实现的一致性由 test_state_adapter 交叉验证。
# ------------------------------------------------------------
add_library(state_adapter SHARED
    state_adapter/src/state_adapter.cpp
    state_adapter/src/state_check.cpp
)
target_include_directories(state_adapter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/state_adapter/include
    ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(state_adapter
    mavlink_units
)

install(TARGETS state_adapter
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
