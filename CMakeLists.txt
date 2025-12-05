cmake_minimum_required(VERSION 3.10)

project(Hydrodrivers VERSION 1.0)

enable_language(C CXX ASM)

set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.deps" CACHE PATH "Base directory for CMake FetchContent downloads")

if(NOT DEFINED MCU_FAMILY)
    message(FATAL_ERROR "MCU_FAMILY variable is not defined. Please set it to F1 or F4.")
endif()

if(NOT MCU_FAMILY STREQUAL "F1" AND NOT MCU_FAMILY STREQUAL "F4")
    message(FATAL_ERROR "Invalid MCU_FAMILY value: ${MCU_FAMILY}. Supported values are: F1, F4")
endif()

message(STATUS "Building for MCU family: ${MCU_FAMILY}")

if(NOT BUILD_TESTS)
    add_subdirectory(third_party)
    add_subdirectory(hydrv_clock)

    # add_subdirectory(hydrv_clock_deprecated)
    add_subdirectory(hydrv_gpio)
    add_subdirectory(hydrv_dma)
    add_subdirectory(hydrv_tim)
    add_subdirectory(hydrv_thruster)
    add_subdirectory(hydrv_vectornav)
    add_subdirectory(hydrv_spi)
    add_subdirectory(hydrv_icm42688)
    add_subdirectory(hydrv_shell/example)
else()
    include(CTest)
endif()

add_subdirectory(hydrv_uart)
add_subdirectory(libs/Hydrolib-soft Hydrolib-soft)
