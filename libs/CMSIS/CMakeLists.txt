cmake_minimum_required(VERSION 3.10)

project(CMakeCMSIS VERSION 1.0)

if(MCU_FAMILY STREQUAL "F1")
    add_library(CMSIS cmsis-device-f1/Source/Templates/system_stm32f1xx.c
                cmsis-device-f1/Source/Templates/gcc/startup_stm32f103xb.s)
    target_include_directories(CMSIS SYSTEM PUBLIC cmsis-device-f1/Include)
elseif(MCU_FAMILY STREQUAL "F4")
    add_library(CMSIS CMSIS_DEVICE_F4/Source/Templates/system_stm32f4xx.c
                CMSIS_DEVICE_F4/Source/Templates/gcc/startup_stm32f407xx.s)
    target_include_directories(CMSIS SYSTEM PUBLIC CMSIS_DEVICE_F4/Include)
else()
    message(FATAL_ERROR "Invalid MCU_FAMILY value in CMSIS: ${MCU_FAMILY}. Supported values are: F1, F4")
endif()

target_include_directories(CMSIS SYSTEM PUBLIC CMSIS_5/CMSIS/Core/Include)
