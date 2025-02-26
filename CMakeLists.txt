cmake_minimum_required(VERSION 3.10)

project(HydrolibRingQueue VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRCS src/*.c)
add_library(Hydrodrivers ${SRCS})

target_include_directories(Hydrodrivers PUBLIC include)
target_include_directories(Hydrodrivers SYSTEM PUBLIC drvs/CMSIS_DEVICE_F4/Include)
target_include_directories(Hydrodrivers SYSTEM PUBLIC drvs/CMSIS_5/CMSIS/Core/Include)
