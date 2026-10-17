# 自动检测目标架构，选择对应的 OrbbecSDK 动态库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(ORBBEC_ARCH x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(ORBBEC_ARCH aarch64)
else()
    message(FATAL_ERROR "OrbbecSDK 不支持当前目标架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(ORBBEC_LIB_DIR
    "${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib/${ORBBEC_ARCH}"
)

add_library(OrbbecSDK SHARED IMPORTED GLOBAL)

set_target_properties(OrbbecSDK PROPERTIES
    IMPORTED_LOCATION
        "${ORBBEC_LIB_DIR}/libOrbbecSDK.so"
    INTERFACE_INCLUDE_DIRECTORIES
        "${CMAKE_SOURCE_DIR}/src/driver/orbbec/include"
)
