cmake_minimum_required(VERSION 3.10)

project(HydrodriversGPIO VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversGPIO src/hydrv_gpio_low.cpp)

target_link_libraries(HydrodriversGPIO PUBLIC HydrolibCommon)
target_link_libraries(HydrodriversGPIO PUBLIC CMSIS)

target_include_directories(HydrodriversGPIO PUBLIC include)

add_subdirectory(example)
