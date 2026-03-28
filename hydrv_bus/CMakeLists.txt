set(LIBRARY_NAME HydrodriversBus)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

target_link_libraries(${LIBRARY_NAME} INTERFACE
    HydrodriversUART INTERFACE HydrodriversGPIO
    INTERFACE HydrolibBusDatalink INTERFACE HydrolibBusApplication)

if(HYDRV_BUILD_EXAMPLES)
    add_subdirectory(example)
endif()
