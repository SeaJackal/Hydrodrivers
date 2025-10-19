cmake_minimum_required(VERSION 3.10)

if(MCU_FAMILY STREQUAL "F1" OR MCU_FAMILY STREQUAL "F4")
    add_subdirectory(upper_half)
    add_subdirectory(lower_half)
    add_subdirectory(example)
    
    message(STATUS "hydrv_uart subdirectories added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "hydrv_uart subdirectories are not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
