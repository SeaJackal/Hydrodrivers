foreach(REQUIRED_VARIABLE SOURCE_DIR BINARY_DIR PRESET MODULES EXPECTED_TEXT)
    if(NOT DEFINED ${REQUIRED_VARIABLE})
        message(FATAL_ERROR "Missing required variable: ${REQUIRED_VARIABLE}")
    endif()
endforeach()

execute_process(
    COMMAND ${CMAKE_COMMAND}
        --preset ${PRESET}
        -B ${BINARY_DIR}
        -DHYDRV_BUILD_MODULES=${MODULES}
    WORKING_DIRECTORY ${SOURCE_DIR}
    RESULT_VARIABLE CONFIGURE_RESULT
    OUTPUT_VARIABLE CONFIGURE_STDOUT
    ERROR_VARIABLE CONFIGURE_STDERR
)

set(CONFIGURE_OUTPUT "${CONFIGURE_STDOUT}\n${CONFIGURE_STDERR}")

if(CONFIGURE_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Configuration unexpectedly succeeded for HYDRV_BUILD_MODULES=${MODULES}"
    )
endif()

string(FIND "${CONFIGURE_OUTPUT}" "${EXPECTED_TEXT}" EXPECTED_TEXT_INDEX)
if(EXPECTED_TEXT_INDEX EQUAL -1)
    message(FATAL_ERROR
        "Configuration failed without expected diagnostic '${EXPECTED_TEXT}'.\n"
        "Output:\n${CONFIGURE_OUTPUT}"
    )
endif()

message(STATUS
    "Configuration failed as expected for HYDRV_BUILD_MODULES=${MODULES}: "
    "${EXPECTED_TEXT}"
)
