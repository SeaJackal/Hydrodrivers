set(TARGET_NAME HydrvMotorExample)

if(MCU_FAMILY STREQUAL "F1" OR MCU_FAMILY STREQUAL "F4" OR MCU_FAMILY STREQUAL "G4")
add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

# target_link_libraries(${TARGET_NAME}
#     HydrodriversClockCPP HydrodriversTimerLow HydrodriversGPIO HydrodriversI2C
#     HydrodriversADCLow HydrolibMath HydrolibMotorController HydrolibPID)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversTimerLow HydrodriversGPIO
    HydrolibMath HydrolibMotorController HydrolibPID HydrodriversADCLow)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
    message(STATUS "${LIBRARY_NAME} target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "${LIBRARY_NAME} target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
