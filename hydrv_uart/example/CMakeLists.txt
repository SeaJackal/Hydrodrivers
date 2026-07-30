set(TARGET_NAME HydrvUARTExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversUART HydrodriversGPIO)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)

get_property(HYDRODRIVERS_RENODE_TEST_HELPER GLOBAL PROPERTY HYDRODRIVERS_RENODE_TEST_HELPER)
include(${HYDRODRIVERS_RENODE_TEST_HELPER})
hydrv_add_renode_tests(${TARGET_NAME} tests/uart_sim_test.py)

set(TARGET_NAME HydrvRSExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main_rs485.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversUART HydrodriversGPIO)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
