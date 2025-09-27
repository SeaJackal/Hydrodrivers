set(TARGET_NAME HydrvTIMERExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversTimerLow HydrodriversGPIO)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
