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

set_target_properties(
    shm_write
    shm_read
    test_camera
    test_math
    test_command
    test_communicate
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)