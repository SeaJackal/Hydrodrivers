if(MCU_FAMILY STREQUAL "F4")
    add_subdirectory(upper_half)
    add_subdirectory(lower_half)

    hydrv_module_is_requested(hydrv_i2c HYDRV_CURRENT_MODULE_REQUESTED)
    if(HYDRV_BUILD_EXAMPLES AND HYDRV_CURRENT_MODULE_REQUESTED)
        add_subdirectory(example)
    endif()

    message(STATUS "HydrodriversI2C target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "HydrodriversI2C target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
