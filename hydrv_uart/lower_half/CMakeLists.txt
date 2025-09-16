cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME HydrodriversUARTLow)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} INTERFACE)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibCommon)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversGPIO)
target_link_libraries(${LIBRARY_NAME} INTERFACE CMSIS)
target_include_directories(${LIBRARY_NAME} INTERFACE include)

if(BUILD_TESTS)
    add_subdirectory(mock)
endif()
