set(TARGET_NAME HydrvADCExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME} HydrodriversClockCPP HydrodriversGPIO HydrodriversADCLow)

