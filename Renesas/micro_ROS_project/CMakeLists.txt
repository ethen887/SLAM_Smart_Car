# Please see README.md for build instructions

cmake_minimum_required(VERSION 3.16.4)

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_SYSTEM_PROCESSOR arm)

# Use add_library() with the STATIC option to name the source file in the generated project.
# This avoids running the linker and is intended for use with cross-compiling toolchains that
# cannot link without custom flags or linker scripts
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo MinSizeRel)

if(BUILD_KIT)
	message(NOTICE "Build Kit is defined as ${BUILD_KIT}")
endif()

message(NOTICE "CMAKE_CURRENT_BINARY_DIR is defined as ${CMAKE_CURRENT_BINARY_DIR}")

get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

if(isMultiConfig)
    message(NOTICE "Multi-config generator(${CMAKE_GENERATOR}) is active")
else()
    message(NOTICE "Single-config generator(${CMAKE_GENERATOR}) is active")
	if(DEFINED CMAKE_BUILD_TYPE)
	 	message(NOTICE "CMAKE_BUILD_TYPE is defined as ${CMAKE_BUILD_TYPE}")
	elseif(DEFINED BUILD_TYPE)
		set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
		message(NOTICE "CMAKE_BUILD_TYPE is set to BUILD_TYPE ENV ${CMAKE_BUILD_TYPE}")
	else()
		message(FATAL_ERROR "Please specify the CMAKE_BUILD_TYPE with -DCMAKE_BUILD_TYPE=")
	endif()
	message(NOTICE "CMAKE_RUNTIME_OUTPUT_DIRECTORY is defined as ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

cmake_minimum_required(VERSION 3.16.4)

# Project configuration
project(micro_ROS_project
	VERSION 1.0.0
	LANGUAGES C CXX ASM)

set(CMAKE_CROSSCOMPILING "TRUE")
set(CMAKE_EXPORT_COMPILE_COMMANDS "TRUE")

include(${CMAKE_CURRENT_LIST_DIR}/Config.cmake)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/GeneratedCfg.cmake)

# 去除 RASC 默认 -Waggregate-return（micro-ROS 头文件按值返回结构体，属误报）
list(REMOVE_ITEM RASC_CMAKE_C_FLAGS "-Waggregate-return")

# If required user can amend RASC generated flags here, e.g. list(REMOVE_ITEM RASC_CMAKE_C_FLAGS "-Wall")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/GeneratedSrc.cmake)

# 自动生成 .hex（Intel HEX）
add_custom_command(
    TARGET
        ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Creating Intel HEX file in ${PROJECT_BINARY_DIR}"
)

# ============================================
# micro-ROS integration
# ============================================

# micro-ROS 静态库根目录
set(MICROROS_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/micro_ros_renesas2estudio_component/libmicroros"
)

# micro-ROS 头文件路径
target_include_directories(${PROJECT_NAME}.elf
    PRIVATE
    "${MICROROS_ROOT}/include"
)

# 导入预编译静态库
add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION
    "${MICROROS_ROOT}/libmicroros.a"
)

# 链接进固件
target_link_libraries(${PROJECT_NAME}.elf
    PRIVATE
    microros
)

# ============================================
# 自动收集 src/ 下所有 .c 文件
# ============================================

# CONFIGURE_DEPENDS: 文件列表变化时自动重配 CMake
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
)

message(NOTICE "==========================================")
message(NOTICE "User sources found:")
foreach(src ${USER_SOURCES})
    message(NOTICE "  ${src}")
endforeach()
message(NOTICE "==========================================")

target_sources(${PROJECT_NAME}.elf
    PRIVATE
    ${USER_SOURCES}
)

# 用户头文件搜索路径（.h 分散在子目录）
target_include_directories(${PROJECT_NAME}.elf
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/src/micro_ros"
)