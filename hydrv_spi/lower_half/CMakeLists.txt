cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME HydroodriversSPILow)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(MCU_FAMILY STREQUAL "F1")
    add_library(${LIBRARY_NAME} INTERFACE)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibCommon)
    target_link_libraries(${LIBRARY_NAME} INTERFACE CMSIS)
    target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversGPIO)
    
    if(MCU_FAMILY STREQUAL "F1")
        target_include_directories(${LIBRARY_NAME} INTERFACE f1/include)
    endif()

    message(STATUS "${LIBRARY_NAME} target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "${LIBRARY_NAME} target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
