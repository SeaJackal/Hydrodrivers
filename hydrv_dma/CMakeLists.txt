cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversDMA INTERFACE)

target_link_libraries(HydrodriversDMA INTERFACE HydrolibCommon)
target_link_libraries(HydrodriversDMA INTERFACE CMSIS)

target_include_directories(HydrodriversDMA INTERFACE include)
