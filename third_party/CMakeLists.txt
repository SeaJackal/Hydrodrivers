set(HYDRV_PROJECT_BUILD_TESTS ${BUILD_TESTS})
set(BUILD_TESTS OFF)
add_subdirectory(Hydrolib)
set(BUILD_TESTS ${HYDRV_PROJECT_BUILD_TESTS})
unset(HYDRV_PROJECT_BUILD_TESTS)

if(NOT BUILD_TESTS)
    if(MCU_FAMILY STREQUAL "F1")
        add_subdirectory(cmsis-device-f1)
    elseif(MCU_FAMILY STREQUAL "F4")
        add_subdirectory(cmsis-device-f4)
    else()
        message(FATAL_ERROR "Invalid MCU_FAMILY value in CMSIS: ${MCU_FAMILY}. Supported values are: F1, F4")
    endif()

    add_subdirectory(CMSIS_5)

    add_library(CMSIS INTERFACE)

    target_link_libraries(CMSIS INTERFACE CMSIS-device INTERFACE CMSIS_5)
endif()
