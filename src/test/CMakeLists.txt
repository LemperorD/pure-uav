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

add_executable(test_math
    test_common/test_math.cpp
)
target_include_directories(test_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)
target_link_libraries(test_math
    pure_math_tools
)

# 制导模块单元测试（讲义 §10 手算例 + 三条铁律 + 三律端到端）
add_executable(test_guidance
    test_guidance/test_guidance.cpp
)
target_include_directories(test_guidance PUBLIC
    ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(test_guidance
    guidance_libs
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
#   链接要求：state_adapter（适配层）+ mavlink_units（单位换算）
#             + guidance_libs（交叉验证 quat_rotate_vec，并调用 guidance_step 校验不变量）
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
    guidance_libs
)

set_target_properties(
    shm_write
    shm_read
    test_camera
    test_guidance
    test_mavlink_units
    test_state_adapter
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)