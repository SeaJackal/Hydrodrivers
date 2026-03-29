set(TARGET_NAME HydrvI2CExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversI2C HydrodriversUART HydrolibLogger)
