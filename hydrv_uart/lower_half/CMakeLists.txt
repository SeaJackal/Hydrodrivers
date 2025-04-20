cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversUARTLow INTERFACE)

target_link_libraries(HydrodriversUARTLow INTERFACE HydrolibCommon)
target_link_libraries(HydrodriversUARTLow INTERFACE HydrodriversGPIO)
target_link_libraries(HydrodriversUARTLow INTERFACE CMSIS)

target_include_directories(HydrodriversUARTLow INTERFACE include)

if(BUILD_TESTING)
    add_subdirectory(mock)
endif()
