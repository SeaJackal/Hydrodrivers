set(HYDRV_MODULE_ORDER
    hydrv_clock
    hydrv_gpio
    hydrv_dma
    hydrv_tim
    hydrv_thruster
    hydrv_uart
    hydrv_vectornav
    hydrv_spi
    hydrv_i2c
    hydrv_icm42688
    hydrv_shell
    hydrv_bus
)

# Keep this registry in sync with target_link_libraries in every module and
# example. The resolver needs this information before add_subdirectory() makes
# the CMake targets available.
set(HYDRV_MODULE_hydrv_clock_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_clock_DEPENDENCIES "")
set(HYDRV_MODULE_hydrv_clock_EXAMPLE_FAMILIES "")
set(HYDRV_MODULE_hydrv_clock_EXAMPLE_DEPENDENCIES "")
set(HYDRV_MODULE_hydrv_clock_TARGETS HydrodriversClockCPP)
set(HYDRV_MODULE_hydrv_clock_EXAMPLE_TARGETS "")

set(HYDRV_MODULE_hydrv_gpio_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_gpio_DEPENDENCIES "")
set(HYDRV_MODULE_hydrv_gpio_EXAMPLE_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_gpio_EXAMPLE_DEPENDENCIES hydrv_clock)
set(HYDRV_MODULE_hydrv_gpio_TARGETS HydrodriversGPIO)
set(HYDRV_MODULE_hydrv_gpio_EXAMPLE_TARGETS HydrvGPIOExample)

set(HYDRV_MODULE_hydrv_dma_FAMILIES F4)
set(HYDRV_MODULE_hydrv_dma_DEPENDENCIES "")
set(HYDRV_MODULE_hydrv_dma_EXAMPLE_FAMILIES "")
set(HYDRV_MODULE_hydrv_dma_EXAMPLE_DEPENDENCIES "")
set(HYDRV_MODULE_hydrv_dma_TARGETS HydrodriversDMA)
set(HYDRV_MODULE_hydrv_dma_EXAMPLE_TARGETS "")

set(HYDRV_MODULE_hydrv_tim_FAMILIES F4)
set(HYDRV_MODULE_hydrv_tim_DEPENDENCIES hydrv_gpio)
set(HYDRV_MODULE_hydrv_tim_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_tim_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_gpio)
set(HYDRV_MODULE_hydrv_tim_TARGETS HydrodriversTimerLow)
set(HYDRV_MODULE_hydrv_tim_EXAMPLE_TARGETS HydrvTIMERExample)

set(HYDRV_MODULE_hydrv_thruster_FAMILIES F4)
set(HYDRV_MODULE_hydrv_thruster_DEPENDENCIES hydrv_gpio hydrv_tim)
set(HYDRV_MODULE_hydrv_thruster_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_thruster_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_gpio hydrv_tim)
set(HYDRV_MODULE_hydrv_thruster_TARGETS HydrodriversThruster)
set(HYDRV_MODULE_hydrv_thruster_EXAMPLE_TARGETS HydrvThrusterExample)

set(HYDRV_MODULE_hydrv_uart_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_uart_DEPENDENCIES hydrv_gpio)
set(HYDRV_MODULE_hydrv_uart_EXAMPLE_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_uart_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_gpio)
set(HYDRV_MODULE_hydrv_uart_TARGETS HydrodriversUART HydrodriversUARTLow)
set(HYDRV_MODULE_hydrv_uart_EXAMPLE_TARGETS HydrvUARTExample HydrvRSExample)

set(HYDRV_MODULE_hydrv_vectornav_FAMILIES F4)
set(HYDRV_MODULE_hydrv_vectornav_DEPENDENCIES hydrv_uart)
set(HYDRV_MODULE_hydrv_vectornav_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_vectornav_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_uart)
set(HYDRV_MODULE_hydrv_vectornav_TARGETS HydrodriversVectorNAV)
set(HYDRV_MODULE_hydrv_vectornav_EXAMPLE_TARGETS HydrvVectorNAVExample)

set(HYDRV_MODULE_hydrv_spi_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_spi_DEPENDENCIES hydrv_gpio)
set(HYDRV_MODULE_hydrv_spi_EXAMPLE_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_spi_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_gpio)
set(HYDRV_MODULE_hydrv_spi_TARGETS HydrodriversSPI HydrodriversSPILow)
set(HYDRV_MODULE_hydrv_spi_EXAMPLE_TARGETS HydrvSPIExample)

set(HYDRV_MODULE_hydrv_i2c_FAMILIES F4)
set(HYDRV_MODULE_hydrv_i2c_DEPENDENCIES hydrv_gpio)
set(HYDRV_MODULE_hydrv_i2c_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_i2c_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_uart)
set(HYDRV_MODULE_hydrv_i2c_TARGETS HydrodriversI2C HydrodriversI2CLow)
set(HYDRV_MODULE_hydrv_i2c_EXAMPLE_TARGETS HydrvI2CExample)

set(HYDRV_MODULE_hydrv_icm42688_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_icm42688_DEPENDENCIES hydrv_spi)
set(HYDRV_MODULE_hydrv_icm42688_EXAMPLE_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_icm42688_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_uart)
set(HYDRV_MODULE_hydrv_icm42688_TARGETS HydrodriversICM42688)
set(HYDRV_MODULE_hydrv_icm42688_EXAMPLE_TARGETS HydrvICM42688Example)

set(HYDRV_MODULE_hydrv_shell_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_shell_DEPENDENCIES hydrv_gpio hydrv_uart)
set(HYDRV_MODULE_hydrv_shell_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_shell_EXAMPLE_DEPENDENCIES
    hydrv_clock hydrv_gpio hydrv_thruster hydrv_uart
)
set(HYDRV_MODULE_hydrv_shell_TARGETS HydrodriversShell)
set(HYDRV_MODULE_hydrv_shell_EXAMPLE_TARGETS HydrvShellExample)

set(HYDRV_MODULE_hydrv_bus_FAMILIES F1 F4)
set(HYDRV_MODULE_hydrv_bus_DEPENDENCIES hydrv_gpio hydrv_uart)
set(HYDRV_MODULE_hydrv_bus_EXAMPLE_FAMILIES F4)
set(HYDRV_MODULE_hydrv_bus_EXAMPLE_DEPENDENCIES hydrv_clock hydrv_gpio hydrv_uart)
set(HYDRV_MODULE_hydrv_bus_TARGETS HydrodriversBus)
set(HYDRV_MODULE_hydrv_bus_EXAMPLE_TARGETS HydrvBusExample)

set(HYDRV_BUILD_MODULES "all" CACHE STRING
    "Semicolon-separated hydrv_* modules to build, or 'all'"
)
set_property(CACHE HYDRV_BUILD_MODULES PROPERTY STRINGS all ${HYDRV_MODULE_ORDER})

function(hydrv_module_is_requested MODULE OUTPUT_VARIABLE)
    list(FIND HYDRV_REQUESTED_MODULES "${MODULE}" _hydrv_requested_index)
    if(_hydrv_requested_index EQUAL -1)
        set(${OUTPUT_VARIABLE} FALSE PARENT_SCOPE)
    else()
        set(${OUTPUT_VARIABLE} TRUE PARENT_SCOPE)
    endif()
endfunction()

function(hydrv_module_supports_family MODULE FAMILY OUTPUT_VARIABLE)
    set(_hydrv_families ${HYDRV_MODULE_${MODULE}_FAMILIES})
    list(FIND _hydrv_families "${FAMILY}" _hydrv_family_index)
    if(_hydrv_family_index EQUAL -1)
        set(${OUTPUT_VARIABLE} FALSE PARENT_SCOPE)
    else()
        set(${OUTPUT_VARIABLE} TRUE PARENT_SCOPE)
    endif()
endfunction()

set(_hydrv_raw_requested ${HYDRV_BUILD_MODULES})
list(REMOVE_DUPLICATES _hydrv_raw_requested)
list(LENGTH _hydrv_raw_requested _hydrv_requested_count)
list(FIND _hydrv_raw_requested "all" _hydrv_all_index)

if(NOT _hydrv_all_index EQUAL -1 AND _hydrv_requested_count GREATER 1)
    message(FATAL_ERROR
        "HYDRV_BUILD_MODULES cannot combine 'all' with module names. "
        "Choose 'all' or one or more of: ${HYDRV_MODULE_ORDER}"
    )
endif()

set(HYDRV_REQUESTED_MODULES "")
if(_hydrv_requested_count EQUAL 0 OR NOT _hydrv_all_index EQUAL -1)
    foreach(_hydrv_module IN LISTS HYDRV_MODULE_ORDER)
        hydrv_module_supports_family("${_hydrv_module}" "${MCU_FAMILY}" _hydrv_supported)
        if(_hydrv_supported)
            list(APPEND HYDRV_REQUESTED_MODULES "${_hydrv_module}")
        endif()
    endforeach()
else()
    foreach(_hydrv_module IN LISTS _hydrv_raw_requested)
        list(FIND HYDRV_MODULE_ORDER "${_hydrv_module}" _hydrv_known_index)
        if(_hydrv_known_index EQUAL -1)
            message(FATAL_ERROR
                "Unknown module '${_hydrv_module}' in HYDRV_BUILD_MODULES. "
                "Allowed values: all;${HYDRV_MODULE_ORDER}"
            )
        endif()

        hydrv_module_supports_family("${_hydrv_module}" "${MCU_FAMILY}" _hydrv_supported)
        if(NOT _hydrv_supported)
            message(FATAL_ERROR
                "Module '${_hydrv_module}' does not support MCU_FAMILY=${MCU_FAMILY}. "
                "Supported MCU families for ${_hydrv_module}: "
                "${HYDRV_MODULE_${_hydrv_module}_FAMILIES}"
            )
        endif()
    endforeach()

    foreach(_hydrv_module IN LISTS HYDRV_MODULE_ORDER)
        list(FIND _hydrv_raw_requested "${_hydrv_module}" _hydrv_requested_index)
        if(NOT _hydrv_requested_index EQUAL -1)
            list(APPEND HYDRV_REQUESTED_MODULES "${_hydrv_module}")
        endif()
    endforeach()
endif()

set(_hydrv_pending_modules ${HYDRV_REQUESTED_MODULES})
set(_hydrv_resolved_unordered "")
list(LENGTH _hydrv_pending_modules _hydrv_pending_count)

while(_hydrv_pending_count GREATER 0)
    list(GET _hydrv_pending_modules 0 _hydrv_module)
    list(REMOVE_AT _hydrv_pending_modules 0)

    list(FIND _hydrv_resolved_unordered "${_hydrv_module}" _hydrv_resolved_index)
    if(_hydrv_resolved_index EQUAL -1)
        hydrv_module_supports_family("${_hydrv_module}" "${MCU_FAMILY}" _hydrv_supported)
        if(NOT _hydrv_supported)
            message(FATAL_ERROR
                "Module '${_hydrv_module}' is required by the selected module set but "
                "does not support MCU_FAMILY=${MCU_FAMILY}"
            )
        endif()

        list(APPEND _hydrv_resolved_unordered "${_hydrv_module}")
        list(APPEND _hydrv_pending_modules ${HYDRV_MODULE_${_hydrv_module}_DEPENDENCIES})

        hydrv_module_is_requested("${_hydrv_module}" _hydrv_is_requested)
        if(HYDRV_BUILD_EXAMPLES AND _hydrv_is_requested)
            set(_hydrv_example_families ${HYDRV_MODULE_${_hydrv_module}_EXAMPLE_FAMILIES})
            list(FIND _hydrv_example_families "${MCU_FAMILY}" _hydrv_example_family_index)
            if(NOT _hydrv_example_family_index EQUAL -1)
                list(APPEND _hydrv_pending_modules
                    ${HYDRV_MODULE_${_hydrv_module}_EXAMPLE_DEPENDENCIES}
                )
            endif()
        endif()
    endif()

    list(LENGTH _hydrv_pending_modules _hydrv_pending_count)
endwhile()

set(HYDRV_RESOLVED_MODULES "")
foreach(_hydrv_module IN LISTS HYDRV_MODULE_ORDER)
    list(FIND _hydrv_resolved_unordered "${_hydrv_module}" _hydrv_resolved_index)
    if(NOT _hydrv_resolved_index EQUAL -1)
        list(APPEND HYDRV_RESOLVED_MODULES "${_hydrv_module}")
    endif()
endforeach()

message(STATUS "Requested Hydrodrivers modules: ${HYDRV_REQUESTED_MODULES}")
message(STATUS "Resolved Hydrodrivers modules: ${HYDRV_RESOLVED_MODULES}")
