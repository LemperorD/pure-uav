# ============================================================
#  制导模块（王乙芃）
#  - guidance_libs : 制导算法库，供 app 主循环调用
#  - pn_sim        : 3D 拦截数值仿真，离线验证用（不进机载）
# ============================================================

add_library(guidance_libs SHARED
    guidance_libs/src/guidance.cpp
    guidance_libs/src/pn_law.cpp
    guidance_libs/src/sensor.cpp
    guidance_libs/src/target.cpp
)
target_include_directories(guidance_libs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/guidance_libs/include
)

# 说明：guidance_libs 自带 vec3.hpp，不依赖 pure_math_tools（后者无向量/矩阵运算）。
# 因此这里**不** link common，制导模块可独立编译/测试（CI 友好）。
# 若将来确实要用 common 的数学函数，再补 target_link_libraries(guidance_libs pure_math_tools)。

# 数值仿真可执行文件
add_executable(pn_sim
    sim/pn_sim.cpp
)
target_link_libraries(pn_sim
    guidance_libs
)
