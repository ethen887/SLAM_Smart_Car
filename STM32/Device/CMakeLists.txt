add_library(device INTERFACE)

target_sources(device
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/mg513.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mg513.h
)

target_include_directories(device
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
