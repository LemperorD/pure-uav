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
  add_library(vio_app_support STATIC camera_service.cpp glue/comm_imu_source.cpp)
  target_include_directories(vio_app_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(vio_app_support PUBLIC communicate pure_vio_runtime)
  target_link_libraries(main PRIVATE vio_app_support)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_VIO=1)
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
