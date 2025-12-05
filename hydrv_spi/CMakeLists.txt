if(MCU_FAMILY STREQUAL "F1")   
    add_subdirectory(upper_half)
    add_subdirectory(lower_half)
    add_subdirectory(example)

    message(STATUS "HydrolibSPI target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "HydrolibSPI target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()