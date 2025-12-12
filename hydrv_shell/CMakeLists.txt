set(LIBRARY_NAME HydrodriversShell)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} INTERFACE)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibShell)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversGPIO HydrodriversUART)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

add_subdirectory(example)