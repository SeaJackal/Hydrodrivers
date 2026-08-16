cmake_minimum_required(VERSION 3.10)

if(MCU_FAMILY STREQUAL "F1" OR MCU_FAMILY STREQUAL "F4")
    hydrv_module_is_requested(hydrv_uart HYDRV_CURRENT_MODULE_REQUESTED)
    add_subdirectory(upper_half)
    add_subdirectory(lower_half)
    if(HYDRV_BUILD_EXAMPLES AND HYDRV_CURRENT_MODULE_REQUESTED)
        add_subdirectory(example)
    endif()
    
    message(STATUS "hydrv_uart subdirectories added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "hydrv_uart subdirectories are not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
