cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME HydrodriversClockCPP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(MCU_FAMILY STREQUAL "F1" OR MCU_FAMILY STREQUAL "F4")
    add_library(${LIBRARY_NAME} ${CMAKE_CURRENT_LIST_DIR}/upper_half/src/hydrv_time.cpp)
    target_link_libraries(${LIBRARY_NAME} PUBLIC HydrolibReturnCodes)
    target_link_libraries(${LIBRARY_NAME} PUBLIC CMSIS)
    target_include_directories(${LIBRARY_NAME} PUBLIC upper_half/include)
    target_link_options(${LIBRARY_NAME} PUBLIC "-Wl,--undefined=_gettimeofday")
    
    if(MCU_FAMILY STREQUAL "F1")
        target_include_directories(${LIBRARY_NAME} PUBLIC f1/include)
    elseif(MCU_FAMILY STREQUAL "F4")
        target_include_directories(${LIBRARY_NAME} PUBLIC f4/include)
    endif()
    
    message(STATUS "${LIBRARY_NAME} target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "${LIBRARY_NAME} target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
