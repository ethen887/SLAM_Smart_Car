add_library(app INTERFACE)

target_sources(app
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/app_move.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_move.h
        ${CMAKE_CURRENT_SOURCE_DIR}/app_task.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_task.h
)

target_include_directories(app
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
