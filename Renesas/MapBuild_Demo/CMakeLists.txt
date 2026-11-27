cmake_minimum_required(VERSION 3.16)
project(MapBuild C)
set(CMAKE_C_STANDARD 99)

set(SLAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../TinySLAM/3.LSLidar/src/Slam")

set(SLAM_CORE_SOURCES
    ${SLAM_DIR}/map_core.c
    ${SLAM_DIR}/slam_scan_accum.c
    ${SLAM_DIR}/slam_matcher.c
    ${SLAM_DIR}/slam_pose_graph.c
    ${SLAM_DIR}/slam_engine.c
)

add_library(mapcore SHARED core/map_core.c)

add_library(mapreal SHARED core/real_wrapper.c ${SLAM_DIR}/map_core.c)
target_include_directories(mapreal PRIVATE ${SLAM_DIR})
set_target_properties(mapreal PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(slamcore SHARED core/slam_wrapper.c ${SLAM_CORE_SOURCES})
target_include_directories(slamcore PRIVATE ${SLAM_DIR})
set_target_properties(slamcore PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(NOT WIN32)
    target_link_libraries(mapcore m)
    target_link_libraries(mapreal m)
    target_link_libraries(slamcore m)
endif()

enable_testing()

add_executable(test_map tests/test_map.c core/map_core.c)
target_include_directories(test_map PRIVATE core)
if(NOT WIN32)
    target_link_libraries(test_map m)
endif()
add_test(NAME map_core COMMAND test_map)

add_executable(test_slam tests/test_slam.c ${SLAM_CORE_SOURCES})
target_include_directories(test_slam PRIVATE ${SLAM_DIR})
if(NOT WIN32)
    target_link_libraries(test_slam m)
endif()
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(slamcore PRIVATE -Wall -Wextra)
    target_compile_options(test_slam PRIVATE -Wall -Wextra)
endif()
add_test(NAME slam_core COMMAND test_slam)
