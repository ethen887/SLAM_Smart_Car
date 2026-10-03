add_library(algorithm INTERFACE)

target_sources(algorithm
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Pid/alg_pid.c
)

target_include_directories(algorithm
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Pid
)