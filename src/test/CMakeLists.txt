set(TEST_OUTPUT_DIR
    ${CMAKE_SOURCE_DIR}/bin/test
)

add_executable(shm_write
    test_shm/sw.cpp
)
add_executable(shm_read
    test_shm/sr.cpp
)

set_target_properties(
    shm_write
    shm_read
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)