cmake_minimum_required(VERSION 3.10)

project(CMakeCMSIS VERSION 1.0)

add_library(CMSIS INTERFACE)

if(BUILD_TESTING)
    target_compile_definitions(CMSIS INTERFACE STM32F407xx)
endif()

target_include_directories(CMSIS SYSTEM INTERFACE CMSIS_DEVICE_F4/Include)
target_include_directories(CMSIS SYSTEM INTERFACE CMSIS_5/CMSIS/Core/Include)
