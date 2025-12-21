if(MCU_FAMILY STREQUAL "F4")

set(TARGET_NAME HydrvBusExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversUART HydrodriversGPIO
    HydrolibBusDatalink HydrolibBusApplication)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)

target_compile_options(HydrvBusExample PRIVATE -fconcepts-diagnostics-depth=10)

message(STATUS "${TARGET_NAME} target added for MCU_FAMILY: ${MCU_FAMILY}")
else()
    message(WARNING "${TARGET_NAME} target is not supported for MCU_FAMILY: ${MCU_FAMILY}")
endif()
