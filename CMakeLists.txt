cmake_minimum_required(VERSION 3.10)

project(Hydrodrivers VERSION 1.0)

add_subdirectory(libs/CMSIS CMSIS)
add_subdirectory(libs/Hydrolib-soft Hydrolib-soft)
add_subdirectory(hydrv_common)
add_subdirectory(hydrv_clock)
add_subdirectory(hydrv_gpio)
add_subdirectory(hydrv_uart)
add_subdirectory(hydrv_dma)
add_subdirectory(hydrv_tim)
add_subdirectory(hydrv_thruster)
# add_subdirectory(hydrv_serial_protocol)
