# 推理后端：
#   AUTO（默认）优先 NPU（VIPLite），找不到 NPU 依赖时自动改用 CPU（ONNX Runtime）
#   NPU / CPU  强制指定后端
#
# PURE_DETECTION_BUILD：
#   AUTO  依赖不全时跳过，整仓库仍可编译
#   ON    依赖不全时 FATAL_ERROR 并列出缺项
#   OFF   不构建

set(PURE_DETECTION_BUILD "AUTO" CACHE STRING
    "Build detection module: AUTO / ON / OFF")
set_property(CACHE PURE_DETECTION_BUILD PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${PURE_DETECTION_BUILD}" _det_build)

set(PURE_DETECTION_BACKEND "AUTO" CACHE STRING
    "Inference backend: AUTO / NPU / CPU")
set_property(CACHE PURE_DETECTION_BACKEND PROPERTY STRINGS AUTO NPU CPU)
string(TOUPPER "${PURE_DETECTION_BACKEND}" _det_backend)

if(_det_build STREQUAL "OFF")
    message(STATUS "detection-rt: PURE_DETECTION_BUILD=OFF，跳过")
    return()
endif()

function(_det_report_missing)
    if(_det_build STREQUAL "ON")
        string(REPLACE ";" "\n  - " _missing_str "${_det_missing}")
        message(FATAL_ERROR
            "detection-rt: 缺少以下依赖（PURE_DETECTION_BUILD=ON）：\n  - ${_missing_str}")
    else()
        message(STATUS "detection-rt: 依赖不全，跳过构建：${_det_missing}")
    endif()
endfunction()

# 公共依赖：OpenCV + 仓库内 OrbbecSDK
set(_det_missing "")
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
    list(APPEND _det_missing "OpenCV 4（find_package(OpenCV 4) 失败）")
endif()

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

if(_det_missing)
    _det_report_missing()
    return()
endif()

# NPU 依赖：VIPLite
set(_npu_missing "")
find_path(VIP_LITE_INCLUDE_DIR vip_lite.h)
if(NOT VIP_LITE_INCLUDE_DIR)
    list(APPEND _npu_missing "vip_lite.h（全志 VIPLite 开发头，通常在 /usr/include）")
endif()

find_library(NBGLINKER_LIBRARY NAMES NBGlinker
             PATHS /lib /usr/lib /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
                   /usr/lib64 /lib64
             NO_DEFAULT_PATH)
if(NOT NBGLINKER_LIBRARY)
    find_library(NBGLINKER_LIBRARY NAMES NBGlinker)
endif()
if(NOT NBGLINKER_LIBRARY)
    list(APPEND _npu_missing "libNBGlinker.so")
endif()

find_library(VIPHAL_LIBRARY NAMES VIPhal
             PATHS /lib /usr/lib /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
                   /usr/lib64 /lib64
             NO_DEFAULT_PATH)
if(NOT VIPHAL_LIBRARY)
    find_library(VIPHAL_LIBRARY NAMES VIPhal)
endif()
if(NOT VIPHAL_LIBRARY)
    list(APPEND _npu_missing "libVIPhal.so")
endif()

# CPU 依赖：ONNX Runtime
set(_ort_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
set(_ort_vendor_lib "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(_ort_vendor_lib "${_ort_root}/lib/aarch64")
endif()
set(_cpu_missing "")
if(EXISTS "${_ort_root}/include/onnxruntime_cxx_api.h" AND
   EXISTS "${_ort_vendor_lib}/libonnxruntime.so")
    set(ORT_INCLUDE_DIR "${_ort_root}/include")
    set(ORT_LIBRARY "${_ort_vendor_lib}/libonnxruntime.so")
else()
    find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h)
    find_library(ORT_LIBRARY NAMES onnxruntime)
endif()
if(NOT ORT_INCLUDE_DIR)
    list(APPEND _cpu_missing "onnxruntime_cxx_api.h（ONNX Runtime 开发头）")
endif()
if(NOT ORT_LIBRARY)
    list(APPEND _cpu_missing "libonnxruntime.so")
endif()

# 选择后端
set(_det_missing "")
if(_det_backend STREQUAL "AUTO")
    if(NOT _npu_missing)
        set(_det_backend_resolved NPU)
    elseif(NOT _cpu_missing)
        message(STATUS "detection-rt: 未检测到 NPU，自动改用 CPU（ONNX Runtime）")
        set(_det_backend_resolved CPU)
    else()
        foreach(_m ${_npu_missing})
            list(APPEND _det_missing "NPU 后端: ${_m}")
        endforeach()
        foreach(_m ${_cpu_missing})
            list(APPEND _det_missing "CPU 后端: ${_m}")
        endforeach()
    endif()
elseif(_det_backend STREQUAL "NPU")
    set(_det_missing ${_npu_missing})
    set(_det_backend_resolved NPU)
elseif(_det_backend STREQUAL "CPU")
    set(_det_missing ${_cpu_missing})
    set(_det_backend_resolved CPU)
else()
    message(FATAL_ERROR
        "detection-rt: PURE_DETECTION_BACKEND 必须是 AUTO / NPU / CPU，当前: ${PURE_DETECTION_BACKEND}")
endif()

if(_det_missing)
    _det_report_missing()
    return()
endif()

if(_det_backend_resolved STREQUAL "NPU")
    message(STATUS "detection-rt: backend=NPU; OpenCV ${OpenCV_VERSION}; "
                   "VIPLite ${VIP_LITE_INCLUDE_DIR}; Orbbec ${_orb_lib_dir}")
else()
    message(STATUS "detection-rt: backend=CPU; OpenCV ${OpenCV_VERSION}; "
                   "ONNX Runtime ${ORT_LIBRARY}; Orbbec ${_orb_lib_dir}")
endif()

# 库
set(_det_sources
    src/camera_source.cpp
    src/yolo_postprocess.cpp
    src/detector.cpp
    src/target_bridge.cpp
)
if(_det_backend_resolved STREQUAL "NPU")
    list(APPEND _det_sources
        src/yolo_npu.cpp
        third_party/awnn/awnn_lib.c
        third_party/awnn/awnn_quantize.c
    )
else()
    list(APPEND _det_sources src/yolo_cpu.cpp)
endif()

add_library(detection_rt STATIC ${_det_sources})

set_target_properties(detection_rt PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(detection_rt
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src/guidance/flight
        ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
        ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(_det_backend_resolved STREQUAL "NPU")
    target_include_directories(detection_rt PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/awnn
        ${VIP_LITE_INCLUDE_DIR}
    )
    target_link_libraries(detection_rt PRIVATE
        ${NBGLINKER_LIBRARY}
        ${VIPHAL_LIBRARY}
    )
    target_compile_definitions(detection_rt PUBLIC PURE_DETECTION_BACKEND_NPU=1)
else()
    target_include_directories(detection_rt PRIVATE ${ORT_INCLUDE_DIR})
    target_link_libraries(detection_rt PRIVATE ${ORT_LIBRARY})
    target_compile_definitions(detection_rt PUBLIC PURE_DETECTION_BACKEND_CPU=1)
endif()

target_link_libraries(detection_rt
    PUBLIC
        ${OpenCV_LIBS}
    PRIVATE
        OrbbecSDK
        pthread
        m
)

# 命令行程序
add_executable(detect_rt
    src/detect_rt_main.cpp
)
target_link_libraries(detect_rt
    PRIVATE
        detection_rt
)

# RPATH：OrbbecSDK；CPU 后端再加 ONNX Runtime 目录
set(_det_rpath "${_orb_lib_dir}")
if(_det_backend_resolved STREQUAL "CPU")
    get_filename_component(_ort_lib_dir "${ORT_LIBRARY}" DIRECTORY)
    list(APPEND _det_rpath "${_ort_lib_dir}")
endif()
set_target_properties(detect_rt PROPERTIES
    BUILD_RPATH "${_det_rpath}"
    INSTALL_RPATH "${_det_rpath}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
