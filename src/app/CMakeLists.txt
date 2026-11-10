add_library(guidance_sim_tools STATIC sim/guidance_sim_tools.cpp)
target_include_directories(guidance_sim_tools
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(guidance_sim_tools PUBLIC guidance_flight)

add_library(guidance_application STATIC startup/guidance_application.cpp)
target_include_directories(guidance_application
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(guidance_application PUBLIC guidance_sim_tools
                                                  guidance_flight)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE communicate guidance_application)
if(TARGET pure_vio_runtime)
  add_library(vio_app_support STATIC camera_service.cpp pose_gate.cpp
                                  glue/comm_imu_source.cpp)
  target_include_directories(
    vio_app_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_SOURCE_DIR}/src/vio/estimator/src)
  target_link_libraries(vio_app_support PUBLIC communicate pure_vio_runtime)
  target_link_libraries(main PRIVATE vio_app_support)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_VIO=1)
endif()

# VIO 接入的无硬件回归；不打开串口或相机。
if(TARGET vio_app_support)
  add_executable(test_pose_gate
                 ${CMAKE_SOURCE_DIR}/src/test/test_app/test_pose_gate.cpp)
  target_link_libraries(test_pose_gate PRIVATE vio_app_support)
  add_test(NAME pose_gate COMMAND test_pose_gate)

  add_executable(test_heartbeat_wait
                 ${CMAKE_SOURCE_DIR}/src/test/test_app/test_heartbeat_wait.cpp)
  target_link_libraries(test_heartbeat_wait PRIVATE vio_app_support)
  add_test(NAME heartbeat_wait COMMAND test_heartbeat_wait)
endif()
if(TARGET detection_rt)
  target_link_libraries(main PRIVATE detection_rt)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_DETECTION=1)
endif()

add_executable(guidance_example startup/guidance_example.cpp)
target_link_libraries(guidance_example PRIVATE guidance_application)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致）， 这样文档里的
# `./bin/main` / `./bin/guidance_example` 在 Windows 与 WSL 上都成立。
set_target_properties(main guidance_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY
${CMAKE_SOURCE_DIR}/bin)
