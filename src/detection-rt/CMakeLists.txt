# detection-rt：先探测 VIPLite，齐全则用 NPU，否则用 ONNX Runtime target。 产物：detection_rt
# 静态库、detect_rt 命令行调试程序。 公共依赖：OpenCV 4 与仓库内按架构分目录的 OrbbecSDK。
find_package(OpenCV 4 QUIET)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(_det_arch x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(_det_arch aarch64)
else()
  set(_det_arch "${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(_det_orb_root "${CMAKE_SOURCE_DIR}/src/driver/orbbec")
set(_det_orb_lib "${_det_orb_root}/lib/${_det_arch}")
if(NOT OpenCV_FOUND
   OR NOT EXISTS "${_det_orb_root}/include/libobsensor/ObSensor.hpp"
   OR NOT EXISTS "${_det_orb_lib}/libOrbbecSDK.so"
)
  message(STATUS "detection-rt: 缺少 OpenCV / OrbbecSDK，跳过")
  return()
endif()

# 检测参数唯一真源 config/detector-rt.yaml 的解析依赖。
find_package(yaml-cpp REQUIRED)

# 后端选择：NPU 优先，三件套缺任一项就退回 CPU。
find_path(VIP_LITE_INCLUDE_DIR vip_lite.h)
find_library(NBGLINKER_LIBRARY NAMES NBGlinker)
find_library(VIPHAL_LIBRARY NAMES VIPhal)
if(VIP_LITE_INCLUDE_DIR
   AND NBGLINKER_LIBRARY
   AND VIPHAL_LIBRARY
)
  set(_det_backend NPU)
else()
  set(_det_backend CPU)
  set(_det_ort_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")

  # 头文件优先用仓库内版本，与板端保持一致。
  if(EXISTS "${_det_ort_root}/include/onnxruntime_cxx_api.h")
    set(ORT_INCLUDE_DIR "${_det_ort_root}/include")
  else()
    find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h)
  endif()

  # 发行版常只装 libonnxruntime.so.1.23 这类带版本号的库， find_library 找不到，故先用 GLOB 枚举候选。
  set(_det_ort_vendor "${_det_ort_root}/lib/${_det_arch}/libonnxruntime.so")
  if(EXISTS "${_det_ort_vendor}")
    set(ORT_LIBRARY "${_det_ort_vendor}")
  else()
    file(GLOB _det_ort_cands
         /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/libonnxruntime.so.*
         /usr/lib64/libonnxruntime.so.* /usr/lib/libonnxruntime.so.*
    )
    if(_det_ort_cands)
      list(GET _det_ort_cands 0 ORT_LIBRARY)
    else()
      find_library(ORT_LIBRARY NAMES onnxruntime)
    endif()
  endif()

  if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIBRARY)
    message(
      FATAL_ERROR
        "detection-rt: NPU 与 CPU 后端均不可用，停止构建。"
        "需要 VIPLite(vip_lite.h / NBGlinker / VIPhal) 或 ONNX Runtime；"
        "ORT_INCLUDE_DIR=${ORT_INCLUDE_DIR}, ORT_LIBRARY=${ORT_LIBRARY}"
    )
  endif()
endif()

message(STATUS "detection-rt: backend=${_det_backend}")

# 后端无关源文件，再按后端追加推理实现。 detection_runtime 是 detect_rt 消费者使用的新运行时；npu_backend 提供
# npuBackendAvailable()/makeNpuBackend()（CPU 构建下走 #else 分支）， input_quantization
# 是配套的量化纯函数。camera_source/detector 仅为遗留测试保留。
set(_det_sources
    src/camera_source.cpp
    src/yolo_postprocess.cpp
    src/detector.cpp
    src/target_bridge.cpp
    src/detection_runtime.cpp
    src/npu_backend.cpp
    src/input_quantization.cpp
    src/detector_config.cpp
)
if(_det_backend STREQUAL "NPU")
  list(APPEND _det_sources src/yolo_npu.cpp third_party/awnn/awnn_lib.c
       third_party/awnn/awnn_quantize.c
  )
else()
  list(APPEND _det_sources src/yolo_cpu.cpp)
endif()

add_library(detection_rt STATIC ${_det_sources})
set_target_properties(
  detection_rt
  PROPERTIES C_STANDARD 11
             C_STANDARD_REQUIRED ON
             POSITION_INDEPENDENT_CODE ON
)

# PUBLIC 会传递给链接者；PRIVATE 只用于本库内部编译。
target_include_directories(
  detection_rt
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
         ${CMAKE_SOURCE_DIR}/src/common
         ${CMAKE_SOURCE_DIR}/src/guidance/flight
         ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
         ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 后端宏必须是 PUBLIC：公开头 yolo_backend.hpp 靠它选 YoloBackend 别名。
if(_det_backend STREQUAL "NPU")
  target_include_directories(
    detection_rt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/awnn
                         ${VIP_LITE_INCLUDE_DIR}
  )
  target_link_libraries(
    detection_rt PRIVATE ${NBGLINKER_LIBRARY} ${VIPHAL_LIBRARY}
  )
  target_compile_definitions(detection_rt PUBLIC PURE_DETECTION_BACKEND_NPU=1)
else()
  target_include_directories(detection_rt PRIVATE ${ORT_INCLUDE_DIR})
  target_link_libraries(detection_rt PRIVATE ${ORT_LIBRARY})
  target_compile_definitions(detection_rt PUBLIC PURE_DETECTION_BACKEND_CPU=1)
endif()

# OpenCV 出现在公开头里，故 PUBLIC；其余只在本库实现里用。 yaml-cpp 只在 detector_config.cpp
# 内部使用，公开头不泄漏其类型。
target_link_libraries(
  detection_rt
  PUBLIC ${OpenCV_LIBS}
  PRIVATE OrbbecSDK pthread m yaml-cpp
)

add_executable(detect_rt src/detect_rt_main.cpp)
# detect_rt 现在是 camera_server 彩色通道的只读消费者，不再打开 Orbbec 设备。
target_link_libraries(detect_rt PRIVATE detection_rt pure_camera_ipc)

# 把 .so 目录写进可执行程序，运行时不必设置 LD_LIBRARY_PATH。
set(_det_rpath "${_det_orb_lib}")
if(_det_backend STREQUAL "CPU")
  get_filename_component(_det_ort_lib_dir "${ORT_LIBRARY}" DIRECTORY)
  list(APPEND _det_rpath "${_det_ort_lib_dir}")
endif()
set_target_properties(
  detect_rt
  PROPERTIES BUILD_RPATH "${_det_rpath}"
             INSTALL_RPATH "${_det_rpath}"
             INSTALL_RPATH_USE_LINK_PATH TRUE
)
