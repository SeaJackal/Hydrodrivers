cmake_minimum_required(VERSION 3.10)

project(HydrodriversGPIO VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.c)
add_library(HydrodriversGPIO ${SRCS})

target_link_libraries(HydrodriversGPIO HydrodriversCommon)
target_link_libraries(HydrodriversGPIO CMSIS)

target_include_directories(HydrodriversGPIO PUBLIC include)
