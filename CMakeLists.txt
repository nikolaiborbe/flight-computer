cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# C/C++ Standards
# ==============================================================================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# Project Setup
# ==============================================================================
set(CMAKE_PROJECT_NAME flight-computer)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
enable_language(C CXX ASM)

set(_dbg_flags "-g3 -gdwarf-4 -O0")

foreach(_cfg DEBUG-WITH-PRECOMPILE DEBUG-WITHOUT-PRECOMPILE)
    set(CMAKE_C_FLAGS_${_cfg}   "${_dbg_flags}" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_${_cfg} "${_dbg_flags}" CACHE STRING "" FORCE)
    set(CMAKE_ASM_FLAGS_${_cfg} "-g3"           CACHE STRING "" FORCE)
endforeach()

# Force re-configure if the CubeMX file changes
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/flight-computer.ioc")

# Includes
include("${CMAKE_SOURCE_DIR}/cmake/constants.cmake")
include("${CMAKE_SOURCE_DIR}/cmake/functions.cmake")
include("${CMAKE_SOURCE_DIR}/cmake/logic.cmake")

# ==============================================================================
# Main Executable (always built from App/vendor sources)
# ==============================================================================
file(GLOB_RECURSE ALL_SOURCES ${SOURCE_DIRS_GLOB_EXPRESSIONS})
apply_global_filters(ALL_SOURCES)
add_executable(${CMAKE_PROJECT_NAME} ${ALL_SOURCES})

# ==============================================================================
# HAL / Middleware: precompiled archive OR fresh CubeMX build
# ==============================================================================
set(USING_PRECOMPILED FALSE)

if (CMAKE_BUILD_TYPE STREQUAL "Debug-with-precompile")
    set(USING_PRECOMPILED TRUE)
    create_stm32_lib()

elseif (CMAKE_BUILD_TYPE STREQUAL "Debug-without-precompile" OR NOT CMAKE_BUILD_TYPE)
    message(STATUS "Building normally without precompiled HAL.")
    add_cube_and_export()  

else()
    message(FATAL_ERROR "Selected build type '${CMAKE_BUILD_TYPE}' does not exist.")
endif()

# ==============================================================================
# App/vendor header include dirs
# ==============================================================================
file(GLOB_RECURSE ALL_HEADERS ${INCLUDE_DIRS_GLOB_EXPRESSIONS})
apply_global_filters(ALL_HEADERS)

set(ALL_INCLUDE_DIRS "")
foreach(HEADER_FILE IN LISTS ALL_HEADERS)
    if(HEADER_FILE MATCHES "^${PATH_VENDOR}")
        get_filename_component(DIR_PATH "${HEADER_FILE}" DIRECTORY)
        list(APPEND ALL_INCLUDE_DIRS "${DIR_PATH}")

    elseif(HEADER_FILE MATCHES "^${PATH_APP}")
        get_filename_component(DIR_PATH "${HEADER_FILE}" DIRECTORY)
        file(RELATIVE_PATH REL_PATH "${PATH_APP}" "${DIR_PATH}")
        if(NOT "${REL_PATH}" STREQUAL "")
            list(APPEND ALL_INCLUDE_DIRS "${PATH_APP}/${REL_PATH}/..")
        endif()
    endif()
endforeach()

if(ALL_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${ALL_INCLUDE_DIRS}
    "${PATH_APP}"
    "${PATH_VENDOR}"
)

# ==============================================================================
# Definitions & link dirs
# ==============================================================================
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)