set(TARGET_NAME HydrvGPIOExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/startup_stm32f407xx.s
    ${CMAKE_CURRENT_LIST_DIR}/src/system_stm32f4xx.c
    ${CMAKE_CURRENT_LIST_DIR}/src/stm32f4xx_it.c)

target_link_libraries(${TARGET_NAME}
    HydrodriversCommon HydrodriversClockCPP HydrodriversGPIO)

target_link_libraries(${TARGET_NAME} HydrolibCommon)

target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
