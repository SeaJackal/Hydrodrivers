cmake_minimum_required(VERSION 3.10)

project(HydrodriversGPIO VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversGPIO INTERFACE)

target_link_libraries(HydrodriversGPIO INTERFACE HydrolibCommon)
target_link_libraries(HydrodriversGPIO INTERFACE CMSIS)

target_include_directories(HydrodriversGPIO INTERFACE include)

add_subdirectory(example)
