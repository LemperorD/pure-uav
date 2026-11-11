add_subdirectory(mission)

add_library(guidance_sim_tools STATIC sim/guidance_sim_tools.cpp)
target_include_directories(
  guidance_sim_tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(guidance_sim_tools PUBLIC guidance_flight)

add_library(guidance_application STATIC startup/guidance_application.cpp)
target_include_directories(
  guidance_application PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(
  guidance_application PUBLIC guidance_sim_tools guidance_flight
)

add_executable(main main.cpp startup/app_options.cpp glue/guidance_session.cpp)
target_link_libraries(
  main PRIVATE communicate guidance_application mission_ports
)
if(TARGET pure_vio_runtime)
  find_package(OpenCV 4 REQUIRED)
  add_library(
    target_adapter STATIC glue/camera_undistort.cpp glue/target_adapter.cpp
  )
  target_include_directories(
    target_adapter
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/common
           ${CMAKE_SOURCE_DIR}/src/guidance/flight ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(target_adapter PUBLIC guidance_flight ${OpenCV_LIBS})

  add_library(
    app_support STATIC app_config.cpp pose_gate.cpp glue/comm_imu_source.cpp
                       glue/mission_service.cpp
  )
  target_include_directories(
    app_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                       ${CMAKE_SOURCE_DIR}/src/vio/estimator/src
  )
  target_link_libraries(
    app_support PUBLIC communicate pure_vio_runtime mission_ports
                       target_adapter
  )
  target_sources(
    main PRIVATE runtime/vio_pose_publisher.cpp startup/app_startup.cpp
                 startup/preflight_config.cpp
  )
  target_link_libraries(main PRIVATE app_support)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_VIO=1)
endif()

# VIO 接入的无硬件回归；不打开串口或相机。
if(TARGET app_support)
  add_executable(
    test_target_adapter
    ${CMAKE_SOURCE_DIR}/src/test/test_app/test_target_adapter.cpp
  )
  target_link_libraries(test_target_adapter PRIVATE target_adapter)
  add_test(NAME target_adapter COMMAND test_target_adapter)

  add_executable(
    test_pose_gate ${CMAKE_SOURCE_DIR}/src/test/test_app/test_pose_gate.cpp
  )
  target_link_libraries(test_pose_gate PRIVATE app_support)
  add_test(NAME pose_gate COMMAND test_pose_gate)

  add_executable(
    test_heartbeat_wait
    ${CMAKE_SOURCE_DIR}/src/test/test_app/test_heartbeat_wait.cpp
  )
  target_link_libraries(test_heartbeat_wait PRIVATE app_support)
  add_test(NAME heartbeat_wait COMMAND test_heartbeat_wait)
endif()
if(TARGET detect_rt)
  # 检测通过 fork/exec 隔离；main 不链接推理库或 Orbbec 采集实现。
  add_dependencies(main detect_rt)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_DETECTION=1)
endif()

add_executable(guidance_example startup/guidance_example.cpp)
target_link_libraries(guidance_example PRIVATE guidance_application)

# 真机流程测试：只链接 communicate，不依赖相机 / VIO / 检测模块。
add_executable(test_drone test_drone.cpp)
target_link_libraries(test_drone PRIVATE communicate)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致）， 这样文档里的
# `./bin/main` / `./bin/guidance_example` / `./bin/test_drone` 在 Windows 与 WSL
# 上都成立。
set_target_properties(
  main guidance_example test_drone PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                              ${CMAKE_SOURCE_DIR}/bin
)

if(TARGET app_support)
  find_package(
    Python3
    COMPONENTS Interpreter
    REQUIRED
  )
  add_test(
    NAME app_config
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_SOURCE_DIR}/src/test/test_app/test_app_config.py
      $<TARGET_FILE:main> ${CMAKE_SOURCE_DIR}
  )
  set_tests_properties(app_config PROPERTIES TIMEOUT 30)
  add_test(
    NAME main_firmware
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_SOURCE_DIR}/src/test/test_app/test_main_firmware.py
      $<TARGET_FILE:main> ${CMAKE_SOURCE_DIR}
  )
  set_tests_properties(main_firmware PROPERTIES TIMEOUT 40)
endif()
