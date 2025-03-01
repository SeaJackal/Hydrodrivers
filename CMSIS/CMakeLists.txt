cmake_minimum_required(VERSION 3.10)

project(CMakeCMSIS VERSION 1.0)

add_library(CMSIS INTERFACE)

target_include_directories(CMSIS SYSTEM INTERFACE CMSIS_DEVICE_F4/Include)
target_include_directories(CMSIS SYSTEM INTERFACE CMSIS_5/CMSIS/Core/Include)
