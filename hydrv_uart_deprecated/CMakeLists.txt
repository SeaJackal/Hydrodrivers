cmake_minimum_required(VERSION 3.10)

project(HydrodriversUART VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.c)
add_library(HydrodriversUART ${SRCS})

target_link_libraries(HydrodriversUART HydrodriversCommon)
target_link_libraries(HydrodriversUART CMSIS)

target_include_directories(HydrodriversUART PUBLIC include)
