cmake_minimum_required(VERSION 3.10)

project(HydrodriversUART VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrodriversUART INTERFACE)

target_link_libraries(HydrodriversUART INTERFACE HydrolibCommon)
target_link_libraries(HydrodriversUART INTERFACE CMSIS)

target_include_directories(HydrodriversUART INTERFACE include)

if(BUILD_TESTING)
    target_include_directories(HydrodriversUART INTERFACE tests/lower_half_mock)
else()
    target_include_directories(HydrodriversUART INTERFACE src/lower_half)
    target_link_libraries(HydrodriversUART INTERFACE HydrodriversGPIO)
endif()

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestUART ${TESTS})
    set_target_properties(TestUART
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestUART PUBLIC cxx_std_20)
    target_compile_options(TestUART PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -g)
    target_link_libraries(TestUART HydrodriversUART GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestUART)
endif()
