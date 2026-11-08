# ============================================================
#  机载 NPU 检测模块（detection_rt / detect_rt）
#
#  依赖：
#    · OpenCV 4（图像解码、letterbox、NMS）
#    · OrbbecSDK（仓库内 src/driver/orbbec，彩色 MJPG/RGB/YUYV）
#    · 板端系统 VIPLite：/usr/include/vip_lite.h
#                       /lib/libNBGlinker.so、/lib/libVIPhal.so
#
#  PURE_DETECTION_BUILD：
#    AUTO（默认）缺任一依赖 → message(STATUS) 跳过，整仓库仍可编译；
#    ON        缺依赖 → FATAL_ERROR 并列出缺项；
#    OFF       不构建。
#
#  awnn 运行时已 vendor 到 third_party/awnn，不依赖 ~/Documents/YOLOtest。
# ============================================================
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

add_library(detection_quantization STATIC src/input_quantization.cpp)
target_include_directories(detection_quantization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(detection_quantization PRIVATE -Wall -Wextra -Wpedantic)

add_library(detection_postprocess STATIC src/yolo_postprocess.cpp)
target_include_directories(detection_postprocess PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(detection_postprocess PUBLIC ${OpenCV_LIBS})
target_compile_options(detection_postprocess PRIVATE -Wall -Wextra -Wpedantic)

add_library(detection_runtime STATIC src/detection_runtime.cpp src/npu_backend.cpp)
target_include_directories(detection_runtime PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src/common)
target_link_libraries(detection_runtime PUBLIC detection_postprocess Threads::Threads)
target_compile_options(detection_runtime PRIVATE -Wall -Wextra -Wpedantic)

set(PURE_DETECTION_BUILD "AUTO" CACHE STRING
    "Build NPU detection module: AUTO / ON / OFF")
set_property(CACHE PURE_DETECTION_BUILD PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${PURE_DETECTION_BUILD}" _det_build)

if(_det_build STREQUAL "OFF")
    target_compile_definitions(detection_runtime PRIVATE PURE_UAV_HAS_NPU=0)
    message(STATUS "detection-rt: PURE_DETECTION_BUILD=OFF，跳过")
    return()
endif()

set(_det_missing "")

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
    list(APPEND _det_missing "OpenCV 4（find_package(OpenCV 4) 失败）")
endif()

# ---- OrbbecSDK（仓库内） ----
set(_orb_root "${CMAKE_SOURCE_DIR}/src/driver/orbbec")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(_orb_arch x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(_orb_arch aarch64)
else()
    set(_orb_arch "${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(_orb_lib_dir "${_orb_root}/lib/${_orb_arch}")
if(NOT EXISTS "${_orb_root}/include/libobsensor/ObSensor.hpp")
    list(APPEND _det_missing "OrbbecSDK 头文件 ${_orb_root}/include/libobsensor/ObSensor.hpp")
endif()
if(NOT EXISTS "${_orb_lib_dir}/libOrbbecSDK.so")
    list(APPEND _det_missing "OrbbecSDK 库 ${_orb_lib_dir}/libOrbbecSDK.so")
endif()

# ---- VIPLite ----
find_path(VIP_LITE_INCLUDE_DIR vip_lite.h)
if(NOT VIP_LITE_INCLUDE_DIR)
    list(APPEND _det_missing "vip_lite.h（全志 VIPLite 开发头，通常在 /usr/include）")
endif()

find_library(NBGLINKER_LIBRARY NAMES NBGlinker
             PATHS /lib /usr/lib /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
                   /usr/lib64 /lib64
             NO_DEFAULT_PATH)
if(NOT NBGLINKER_LIBRARY)
    find_library(NBGLINKER_LIBRARY NAMES NBGlinker)
endif()
if(NOT NBGLINKER_LIBRARY)
    list(APPEND _det_missing "libNBGlinker.so")
endif()

find_library(VIPHAL_LIBRARY NAMES VIPhal
             PATHS /lib /usr/lib /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
                   /usr/lib64 /lib64
             NO_DEFAULT_PATH)
if(NOT VIPHAL_LIBRARY)
    find_library(VIPHAL_LIBRARY NAMES VIPhal)
endif()
if(NOT VIPHAL_LIBRARY)
    list(APPEND _det_missing "libVIPhal.so")
endif()

if(_det_missing)
    if(_det_build STREQUAL "ON")
        string(REPLACE ";" "\n  - " _det_missing_str "${_det_missing}")
        message(FATAL_ERROR
            "detection-rt: PURE_DETECTION_BUILD=ON 但缺少以下依赖：\n  - ${_det_missing_str}")
    else()
        target_compile_definitions(detection_runtime PRIVATE PURE_UAV_HAS_NPU=0)
        message(STATUS "detection-rt: 依赖不全（${_det_build} 模式跳过构建）：${_det_missing}")
        return()
    endif()
endif()

message(STATUS "detection-rt: OpenCV ${OpenCV_VERSION}; VIPLite ${VIP_LITE_INCLUDE_DIR}; "
               "Orbbec ${_orb_lib_dir}")

# ------------------------------------------------------------ 库
add_library(detection_npu STATIC
    src/yolo_npu.cpp
    third_party/awnn/awnn_lib.c
    third_party/awnn/awnn_quantize.c
)
set_target_properties(detection_npu PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(detection_npu
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/awnn
        ${VIP_LITE_INCLUDE_DIR}
)

target_link_libraries(detection_npu
    PUBLIC
        ${OpenCV_LIBS}
        detection_quantization
    PRIVATE
        ${NBGLINKER_LIBRARY}
        ${VIPHAL_LIBRARY}
        pthread
        m
)

target_compile_definitions(detection_runtime PRIVATE PURE_UAV_HAS_NPU=1)
target_link_libraries(detection_runtime PRIVATE detection_npu)

add_library(detection_rt STATIC src/camera_source.cpp src/detector.cpp src/target_bridge.cpp)
target_include_directories(detection_rt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/guidance/flight
    ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
    ${CMAKE_SOURCE_DIR}/src/common/common_libs/include)
target_link_libraries(detection_rt PUBLIC detection_postprocess detection_npu PRIVATE OrbbecSDK Threads::Threads)

# ------------------------------------------------------------ 命令行程序
add_executable(detect_rt
    src/detect_rt_main.cpp
)
target_link_libraries(detect_rt
    PRIVATE
        detection_rt
)

# RPATH：仓库内 OrbbecSDK 动态库 + 板端 NPU 系统库（系统库本就在默认路径）
set_target_properties(detect_rt PROPERTIES
    BUILD_RPATH "${_orb_lib_dir}"
    INSTALL_RPATH "${_orb_lib_dir}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
