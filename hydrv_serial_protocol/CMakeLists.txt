cmake_minimum_required(VERSION 3.10)

project(HydrodriversSerialProtocol VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.cpp)
add_library(HydrodriversSerialProtocol ${SRCS})

target_link_libraries(HydrodriversSerialProtocol HydrodriversUART)

target_link_libraries(HydrodriversSerialProtocol HydrolibSerialProtocol)

target_include_directories(HydrodriversSerialProtocol PUBLIC include)
