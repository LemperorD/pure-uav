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

set_target_properties(
    shm_write
    shm_read
    test_camera
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)