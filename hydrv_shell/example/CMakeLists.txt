if(MCU_FAMILY STREQUAL "F4")   
    set(TARGET_NAME HydrvShellExample)

    add_executable(${TARGET_NAME}
        ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)

    target_link_libraries(${TARGET_NAME}
        HydrodriversClockCPP HydrodriversUART HydrodriversGPIO
        HydrodriversThruster HydrolibShell HydrolibShellCommands
        HydrodriversShell)

    target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
endif()
