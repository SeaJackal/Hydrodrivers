if(MCU_FAMILY STREQUAL "F4")
    set(LIBRARY_NAME HydrodriversVectorNAV)

    add_library(${LIBRARY_NAME} INTERFACE)
    if(HYDRV_BUILD_EXAMPLES)
        add_subdirectory(example)
    endif()

    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibVectorNAV)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversUART)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibLogger)

    target_include_directories(${LIBRARY_NAME} INTERFACE include)

    message(STATUS "hydrv_vectornav subdirectories added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "hydrv_vectornav subdirectories are not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()