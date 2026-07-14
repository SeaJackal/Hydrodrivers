set(LIBRARY_NAME HydrodriversI2C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} INTERFACE)
    
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversI2CLow)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibConcepts)
target_link_libraries(${LIBRARY_NAME} INTERFACE HydrodriversGPIO)
    
target_include_directories(${LIBRARY_NAME} INTERFACE include)
