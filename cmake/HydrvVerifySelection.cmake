foreach(HYDRV_VERIFY_MODULE IN LISTS HYDRV_MODULE_ORDER)
    list(FIND HYDRV_RESOLVED_MODULES "${HYDRV_VERIFY_MODULE}" HYDRV_VERIFY_RESOLVED_INDEX)

    foreach(HYDRV_VERIFY_TARGET IN LISTS HYDRV_MODULE_${HYDRV_VERIFY_MODULE}_TARGETS)
        if(HYDRV_VERIFY_RESOLVED_INDEX EQUAL -1)
            if(TARGET ${HYDRV_VERIFY_TARGET})
                message(FATAL_ERROR
                    "Unexpected target '${HYDRV_VERIFY_TARGET}' exists for unresolved "
                    "module '${HYDRV_VERIFY_MODULE}'"
                )
            endif()
        elseif(NOT TARGET ${HYDRV_VERIFY_TARGET})
            message(FATAL_ERROR
                "Expected target '${HYDRV_VERIFY_TARGET}' is missing for resolved "
                "module '${HYDRV_VERIFY_MODULE}'"
            )
        endif()
    endforeach()

    hydrv_module_is_requested("${HYDRV_VERIFY_MODULE}" HYDRV_VERIFY_REQUESTED)
    set(HYDRV_VERIFY_EXPECT_EXAMPLES FALSE)
    if(HYDRV_BUILD_EXAMPLES AND HYDRV_VERIFY_REQUESTED)
        set(HYDRV_VERIFY_EXAMPLE_FAMILIES
            ${HYDRV_MODULE_${HYDRV_VERIFY_MODULE}_EXAMPLE_FAMILIES}
        )
        list(FIND HYDRV_VERIFY_EXAMPLE_FAMILIES "${MCU_FAMILY}"
            HYDRV_VERIFY_EXAMPLE_FAMILY_INDEX
        )
        if(NOT HYDRV_VERIFY_EXAMPLE_FAMILY_INDEX EQUAL -1)
            set(HYDRV_VERIFY_EXPECT_EXAMPLES TRUE)
        endif()
    endif()

    foreach(HYDRV_VERIFY_TARGET IN LISTS HYDRV_MODULE_${HYDRV_VERIFY_MODULE}_EXAMPLE_TARGETS)
        if(HYDRV_VERIFY_EXPECT_EXAMPLES)
            if(NOT TARGET ${HYDRV_VERIFY_TARGET})
                message(FATAL_ERROR
                    "Expected example target '${HYDRV_VERIFY_TARGET}' is missing for "
                    "requested module '${HYDRV_VERIFY_MODULE}'"
                )
            endif()
        elseif(TARGET ${HYDRV_VERIFY_TARGET})
            message(FATAL_ERROR
                "Unexpected example target '${HYDRV_VERIFY_TARGET}' exists for module "
                "'${HYDRV_VERIFY_MODULE}'"
            )
        endif()
    endforeach()
endforeach()

if(BUILD_TESTS)
    hydrv_module_is_requested(hydrv_uart HYDRV_VERIFY_UART_REQUESTED)
    if(HYDRV_VERIFY_UART_REQUESTED)
        if(NOT TARGET HydrodriversUARTLowMock OR NOT TARGET TestUART)
            message(FATAL_ERROR "Host UART test targets are missing")
        endif()
    elseif(TARGET HydrodriversUARTLowMock OR TARGET TestUART)
        message(FATAL_ERROR "Host UART test targets exist without explicit hydrv_uart selection")
    endif()
endif()

message(STATUS "Hydrodrivers module selection verification passed")
