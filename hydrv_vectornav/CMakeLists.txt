cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME HydrodriversVectorNAV)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} INTERFACE)
add_subdirectory(example)

target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibVectorNAV)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversUART)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibLogger)

target_include_directories(${LIBRARY_NAME} INTERFACE include)
