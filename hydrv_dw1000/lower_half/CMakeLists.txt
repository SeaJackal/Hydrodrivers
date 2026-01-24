cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME HydrodriversDW1000Low)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(MCU_FAMILY STREQUAL "F1")
    add_library(${LIBRARY_NAME} INTERFACE)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversSPI)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibReturnCodes)
    target_include_directories(${LIBRARY_NAME} INTERFACE include)

    if(HYDRV_BUILD_EXAMPLES)
        add_subdirectory(example)
    endif()

    message(STATUS "${LIBRARY_NAME} target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "${LIBRARY_NAME} target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
