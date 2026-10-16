# ============================================================
#  接口适配层（解析侧）—— 王乙芃
#  - mavlink_units : MAVLink 单位换算（纯函数、零依赖）
#
#  职责：只做"数值 + 单位"的换算 —— 不解析报文、不做坐标变换（NED/ENU 属另一模块）。
#  独立成库的理由：
#    ① 每个换算只有一处实现，改一次全对；
#    ② 取整 / 饱和 / 回绕 / 哨兵这些边界情况可以集中到一处并单独测；
#    ③ 解析代码退化成"读字段 → 调函数"，可读性大幅提升。
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
