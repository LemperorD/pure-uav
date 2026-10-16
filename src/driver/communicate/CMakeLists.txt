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

find_package(yaml-cpp REQUIRED)

find_package(Threads REQUIRED)
target_link_libraries(communicate PUBLIC Threads::Threads yaml-cpp)
