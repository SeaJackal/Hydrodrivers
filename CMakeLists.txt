cmake_minimum_required(VERSION 3.10)

project(Hydrodrivers VERSION 1.0)

add_subdirectory(CMSIS)
add_subdirectory(hydrv_common)
add_subdirectory(hydrv_clock)
add_subdirectory(hydrv_gpio)
add_subdirectory(hydrv_uart)
