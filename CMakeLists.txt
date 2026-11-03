cmake_minimum_required(VERSION 3.22)
project(pure-uav)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_SOURCE_DIR}/bin
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${CMAKE_SOURCE_DIR}/lib
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_SOURCE_DIR}/lib
)

enable_testing()

add_subdirectory(src)

# ==================== wifi / PWLink 联通测试 ====================
# 纯 C 上位机程序：手写 MAVLink 2 握手 + 遥测（main.c + mavlink2.c）
add_executable(pwlink_test
    wifi/main.c
    wifi/mavlink2.c
)
target_include_directories(pwlink_test PRIVATE
    ${CMAKE_SOURCE_DIR}/wifi
)
# 代码依赖 clock_gettime / struct timespec 等 POSIX 接口，
# 需要 GNU C99（-std=gnu99）并显式声明 POSIX.1-2008 特性。
set_target_properties(pwlink_test PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
target_compile_definitions(pwlink_test PRIVATE
    _POSIX_C_SOURCE=200809L
)
target_compile_options(pwlink_test PRIVATE -Wall -Wextra)