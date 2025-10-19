set(TARGET_NAME HydrvThrusterExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/startup_stm32f407xx.s
    ${CMAKE_CURRENT_LIST_DIR}/src/system_stm32f4xx.c)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversTimerLow HydrodriversGPIO HydrodriversThruster)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
