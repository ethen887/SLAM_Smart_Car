
cmake_minimum_required(VERSION 3.22)

# C language settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# STM32F103 project
set(CMAKE_PROJECT_NAME STM32_F103)

# Generate compile_commands.json for IntelliSense / clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

enable_language(C ASM)

# Main executable
add_executable(${CMAKE_PROJECT_NAME})

# STM32CubeMX generated code
add_subdirectory(cmake/stm32cubemx)

# User modules
add_subdirectory(Algorithm)
#add_subdirectory(Bsp)
add_subdirectory(Device)
add_subdirectory(App)

# Additional library search paths
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

# Additional source files
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/LSM6DSR/lsm6dsr_reg.c    # ST官方LSM6DSR寄存器驱动
)

# Additional include directories
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/LSM6DSR
)

# Additional compile definitions
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

# Remove incorrect default library dependency
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Link CubeMX and user modules
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    algorithm
    #bsp
    device
    app
)
