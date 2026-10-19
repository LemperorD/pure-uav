set(TEST_OUTPUT_DIR
    ${CMAKE_SOURCE_DIR}/bin/test
)

add_executable(shm_write
    test_common/sw.cpp
)
add_executable(shm_read
    test_common/sr.cpp
)

add_executable(test_camera
    test_driver/test_camera.cpp
)
target_include_directories(test_camera PUBLIC
    ${CMAKE_SOURCE_DIR}/src/driver/orbbec/include
)
target_link_libraries(test_camera
    # ${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib/libOrbbecSDK.so
    OrbbecSDK
)

message(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_math
    test_common/test_math.cpp
)
target_include_directories(test_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)
target_link_libraries(test_math
    pure_math_tools
)

# 制导模块单元测试（讲义 §10 手算例 + 三条铁律 + 三律端到端 + §15 不变量）
#   ⚠ 本测试**确实需要仿真**：它用 TruthState / SensorConfig / Sensor 三级逼真度
#     以及 TargetModel 做端到端，所以链接 guidance_sim（= core + sim）。
add_executable(test_guidance
    test_guidance/test_guidance.cpp
)
target_include_directories(test_guidance PUBLIC
    ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(test_guidance
    guidance_core
    guidance_sim
)

# MAVLink 单位换算单元测试（解析侧接口适配层）
add_executable(test_mavlink_units
    test_interface/test_mavlink_units.cpp
)
target_include_directories(test_mavlink_units PUBLIC
    ${CMAKE_SOURCE_DIR}/src/interface/mavlink_units/include
)
target_link_libraries(test_mavlink_units
    mavlink_units
)

# 数据对齐适配层单元测试（外部口径 → 内部口径）
#   覆盖：NED↔ENU / 时间对齐与回绕 / 姿态（外部机体系→光轴系）/ 四元数交叉验证
#         / 组装 GuidanceState / 上游模拟器（多口径同一场景）/ 不变量自检
#   ★ 链接要求：state_adapter + mavlink_units + **guidance_core**
#     —— 本测试属于**真机路径**，只该依赖核心。它需要 guidance_step() 校验不变量、
#        以及 quat_rotate_vec 做交叉验证；这两样都在 guidance_core 里。
#     ⚠ 不要链接 guidance_sim / guidance_libs —— 那会把仿真拖进真机路径的依赖，
#        正是 2026-10-16 把四元数工具从 sensor.* 移出核心所要修掉的问题。
add_executable(test_state_adapter
    test_interface/test_state_adapter.cpp
)
target_include_directories(test_state_adapter PUBLIC
    ${CMAKE_SOURCE_DIR}/src/interface/state_adapter/include
    ${CMAKE_SOURCE_DIR}/src/interface/mavlink_units/include
    ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(test_state_adapter
    state_adapter
    mavlink_units
    guidance_core
)

add_executable(test_command
    test_driver/test_command.cpp
)
target_include_directories(test_command PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_command
    communicate
)

add_executable(test_communicate
    test_driver/test_communicate.cpp
)
target_include_directories(test_communicate PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_communicate
    communicate
)

add_executable(test_frequency
    test_driver/test_frequency.cpp
)
target_include_directories(test_frequency PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_frequency
    communicate
)

set_target_properties(
    shm_write
    shm_read
    test_camera
    test_guidance
    test_mavlink_units
    test_state_adapter
    test_math
    test_command
    test_communicate
    test_frequency
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)