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

set_target_properties(
    shm_write
    shm_read
    test_camera
    test_guidance
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)