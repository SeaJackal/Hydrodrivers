cmake_minimum_required(VERSION 3.10)

project(HydrodriversUART VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversUART INTERFACE)

target_link_libraries(HydrodriversUART INTERFACE HydrodriversGPIO INTERFACE HydrolibCommon)
target_link_libraries(HydrodriversUART INTERFACE CMSIS)

target_include_directories(HydrodriversUART INTERFACE include)
