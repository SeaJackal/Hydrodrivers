set(LIBRARY_NAME HydrodriversBus)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

target_link_libraries(${LIBRARY_NAME} INTERFACE
    HydrodriversUART INTERFACE HydrodriversGPIO
    INTERFACE HydrolibBusDatalink INTERFACE HydrolibBusApplication)

hydrv_module_is_requested(hydrv_bus HYDRV_CURRENT_MODULE_REQUESTED)
if(HYDRV_BUILD_EXAMPLES AND HYDRV_CURRENT_MODULE_REQUESTED)
    add_subdirectory(example)
endif()
