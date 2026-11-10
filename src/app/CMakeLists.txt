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
if(TARGET vio_camera)
  target_link_libraries(main PRIVATE vio_camera)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_VIO=1)
endif()
if(TARGET detection_rt)
  target_link_libraries(main PRIVATE detection_rt)
  target_compile_definitions(main PRIVATE PURE_APP_HAS_DETECTION=1)
endif()

add_executable(guidance_example startup/guidance_example.cpp)
target_link_libraries(guidance_example PRIVATE guidance_application)

# 真机流程测试：只链接 communicate，不依赖相机 / VIO / 检测模块。
add_executable(test_drone test_drone.cpp)
target_link_libraries(test_drone PRIVATE communicate)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致）， 这样文档里的
# `./bin/main` / `./bin/guidance_example` / `./bin/test_drone` 在 Windows 与 WSL 上都成立。
set_target_properties(main guidance_example test_drone PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
