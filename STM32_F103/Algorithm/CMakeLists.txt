add_library(algorithm INTERFACE)

target_sources(algorithm
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Pid/alg_pid.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Filter/alg_lowpass.c
)

target_include_directories(algorithm
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Pid
        ${CMAKE_CURRENT_SOURCE_DIR}/Filter
)
