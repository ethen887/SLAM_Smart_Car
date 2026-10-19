add_library(device INTERFACE)

target_sources(device
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/dev_mg513.c
        ${CMAKE_CURRENT_SOURCE_DIR}/dev_mg513.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dev_ps2.c
        ${CMAKE_CURRENT_SOURCE_DIR}/dev_ps2.h
        )

target_include_directories(device
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
