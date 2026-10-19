# ============================================================
#  制导模块（王乙芃）
#
#  ★ 核心 / 仿真拆成两个 target —— 保证机上不带仿真代码
#
#    guidance_core : 制导算法核心（真机 + 仿真**共用**，零仿真依赖）
#                      · guidance.cpp  制导主流程（纯函数 guidance_step）
#                      · pn_law.cpp    PPN / TPN / APN + ω / Vc
#                      · quat.cpp      四元数通用数学（见 quat.hpp 的说明）
#                    ⇒ **真机主循环只该链接它**
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
#    依赖方向（2026-10-16 审计确认，无反向依赖）：
#        vec3.hpp ─⌐
#        guidance_types.hpp ─ guidance.hpp ─ guidance.cpp   ← core
#        pn_law.hpp ─ pn_law.cpp                            ← core
#        quat.hpp ─ quat.cpp                                ← core
#        sensor.hpp/cpp ─→ core                             ← sim（单向）
#        target.hpp/cpp ─→ vec3.hpp                         ← sim
# ============================================================

add_library(guidance_core SHARED
    guidance_libs/src/guidance.cpp
    guidance_libs/src/pn_law.cpp
    guidance_libs/src/quat.cpp
)
target_include_directories(guidance_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/guidance_libs/include
)

# 说明：guidance_core 自带 vec3.hpp，不依赖 pure_math_tools（后者无向量/矩阵运算）。
# 因此这里**不** link common，制导模块可独立编译/测试（CI 友好）。

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

# 数值仿真可执行文件（离线验证用，**不进机载**）
add_executable(pn_sim
    sim/pn_sim.cpp
)
target_link_libraries(pn_sim
    guidance_sim
)
