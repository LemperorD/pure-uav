# communicate 模块：飞控 PX4 的 MAVLink 通信库

add_library(communicate SHARED
    src/serial_port.cpp
    src/communicator.cpp
)

target_include_directories(communicate PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(communicate SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink
)

target_compile_features(communicate PUBLIC cxx_std_17)

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/yaml-cpp)

find_package(Threads REQUIRED)
target_link_libraries(communicate PUBLIC Threads::Threads yaml-cpp)
