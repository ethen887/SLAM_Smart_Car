add_library(app INTERFACE)

option(NAVIGATION_MOTION_BENCH_TEST "Build a bench image with a 10 s navigation command timeout" OFF)
if(NAVIGATION_MOTION_BENCH_TEST)
    target_compile_definitions(app INTERFACE NAVIGATION_MOTION_TIMEOUT_MS=10000U)
    message(STATUS "Navigation motion watchdog: 10000 ms (bench test)")
endif()

# F103C8 Flash 有限，IMU、里程计及运动任务按尺寸优化。
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/app_navigation_motion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_test_run.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_host_comm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/host_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_attitude.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_odometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_filter.c
    DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTIES COMPILE_OPTIONS "-Os")

target_sources(app
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/app_navigation_motion.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_host_comm.c
        ${CMAKE_CURRENT_SOURCE_DIR}/host_protocol.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_move.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_move.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_task.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_task.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_log.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_log.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_filter.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_imu_filter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_attitude.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_attitude.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_odometry.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_odometry.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_test_run.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_test_run.h
)

target_include_directories(app
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
