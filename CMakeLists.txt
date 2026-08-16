cmake_minimum_required(VERSION 3.10)

project(Hydrodrivers VERSION 1.0)

enable_language(C CXX ASM)

set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.deps" CACHE PATH "Base directory for CMake FetchContent downloads")
set(RUN_TESTS "ON" CACHE BOOL "Register Renode tests")

option(HYDRV_BUILD_EXAMPLES "Build Hydrodrivers examples" OFF)
option(BUILD_TESTS "Build host unit tests" OFF)
option(HYDRV_VERIFY_SELECTION "Verify the resolved Hydrodrivers target set" OFF)

include(CTest)

if(BUILD_TESTS OR RUN_TESTS)
    enable_testing()
endif()

if(NOT DEFINED MCU_FAMILY)
    message(FATAL_ERROR "MCU_FAMILY variable is not defined. Please set it to F1 or F4.")
endif()

if(NOT MCU_FAMILY STREQUAL "F1" AND NOT MCU_FAMILY STREQUAL "F4")
    message(FATAL_ERROR "Invalid MCU_FAMILY value: ${MCU_FAMILY}. Supported values are: F1, F4")
endif()

message(STATUS "Building for MCU family: ${MCU_FAMILY}")

include(cmake/HydrvModules.cmake)

add_subdirectory(third_party)

if(HYDRV_BUILD_EXAMPLES)
    add_subdirectory(renode)
endif()

foreach(HYDRV_MODULE IN LISTS HYDRV_RESOLVED_MODULES)
    add_subdirectory(${HYDRV_MODULE})
endforeach()

if(HYDRV_VERIFY_SELECTION)
    include(cmake/HydrvVerifySelection.cmake)
endif()

# add_custom_target(compdb ALL
#     COMMAND /bin/sh -c "compdb -p ${CMAKE_BINARY_DIR} list > compile_commands.json && mv compile_commands.json ${CMAKE_BINARY_DIR}/compile_commands.json"
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     VERBATIM
# )
