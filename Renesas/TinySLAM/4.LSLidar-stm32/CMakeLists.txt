# Please see README.md for build instructions

cmake_minimum_required(VERSION 3.16.4)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

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

project(LSLidar
    VERSION 1.0.0
    LANGUAGES C CXX ASM)

set(CMAKE_CROSSCOMPILING "TRUE")
set(CMAKE_EXPORT_COMPILE_COMMANDS "TRUE")

include(${CMAKE_CURRENT_LIST_DIR}/Config.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/GeneratedCfg.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/GeneratedSrc.cmake)

# ===== 以下是用户添加：每次链接完自动生成 HEX 烧录文件 =====
add_custom_command(
    TARGET ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Creating Intel HEX file"
)

# ===== 自动扫描 src/ 下所有 .c 文件，新增文件无需再改 CMake =====
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

target_sources(${PROJECT_NAME}.elf PRIVATE ${USER_SOURCES})

# ===== 自动把 src/ 下所有含头文件的文件夹加入搜索路径 =====
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
foreach(hdr ${USER_HEADERS})
    get_filename_component(hdr_dir ${hdr} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${hdr_dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)
target_include_directories(${PROJECT_NAME}.elf PRIVATE ${USER_INCLUDE_DIRS})