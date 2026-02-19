set(TARGET_NAME HydrvDW1000LowExample)

add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversDW1000Low HydrolibLogger HydrodriversUART)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
