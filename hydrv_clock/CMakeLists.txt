cmake_minimum_required(VERSION 3.10)

project(HydrodriversClockCPP VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

target_link_libraries(HydrodriversClockCPP HydrodriversCommon)
target_link_libraries(HydrodriversClockCPP CMSIS)

target_include_directories(HydrodriversClockCPP PUBLIC include)
