set(LIBRARY_NAME HydrodriversShell)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} INTERFACE)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibShell)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversGPIO HydrodriversUART)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

hydrv_module_is_requested(hydrv_shell HYDRV_CURRENT_MODULE_REQUESTED)
if(HYDRV_BUILD_EXAMPLES AND HYDRV_CURRENT_MODULE_REQUESTED)
    add_subdirectory(example)
endif()
