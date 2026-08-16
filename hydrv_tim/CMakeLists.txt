cmake_minimum_required(VERSION 3.10)

if(MCU_FAMILY STREQUAL "F4")
    # add_subdirectory(upper_half)
    add_subdirectory(lower_half)

    hydrv_module_is_requested(hydrv_tim HYDRV_CURRENT_MODULE_REQUESTED)
    if(HYDRV_BUILD_EXAMPLES AND HYDRV_CURRENT_MODULE_REQUESTED)
        add_subdirectory(example)
    endif()
    
    message(STATUS "hydrv_tim subdirectories added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "hydrv_tim subdirectories are not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
