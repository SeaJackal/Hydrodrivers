cmake_minimum_required(VERSION 3.10)

project(HydrodriversClock VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.c)
add_library(HydrodriversClock ${SRCS})

target_link_libraries(HydrodriversClock HydrodriversCommon)
target_link_libraries(HydrodriversClock CMSIS)

target_include_directories(HydrodriversClock PUBLIC include)
