# camera_server：唯一 Orbbec 采集进程。
# vio_camera 只在 PURE_VIO_BUILD_LIVE 启用（且能找到 OrbbecSDK/MAVLink）时存在，
# 因此本目录必须放在 add_subdirectory(vio/estimator) 之后。
if(NOT TARGET vio_camera)
  message(STATUS "camera_server: skipped (vio_camera target unavailable)")
  return()
endif()

find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(camera_server camera_server_main.cpp frame_publisher.cpp)
target_include_directories(
  camera_server
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_SOURCE_DIR}/src/common
          ${CMAKE_SOURCE_DIR}/src/vio/estimator/src
          ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_server PRIVATE pure_camera_ipc vio_camera
                                           ${OpenCV_LIBS} yaml-cpp)

# 与 vio_live 一致：把仓库内 OrbbecSDK 目录写进 RPATH，运行时无需 LD_LIBRARY_PATH。
set(_orbbec_rpath
    "${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib")
set_target_properties(
  camera_server
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
             BUILD_RPATH "${_orbbec_rpath}" INSTALL_RPATH "${_orbbec_rpath}")
