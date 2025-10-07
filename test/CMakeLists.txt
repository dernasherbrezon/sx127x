cmake_minimum_required(VERSION 3.5)
project(sx127x_test C)

set(CMAKE_C_STANDARD 99)
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
endif()

add_library(sx127xlib
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sx127x.c
)

find_package(PkgConfig REQUIRED)

enable_testing()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/unity-2.5.2/src/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(test_sx127x 
    ${CMAKE_CURRENT_SOURCE_DIR}/test_sx127x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sx127x_mock_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/unity-2.5.2/src/unity.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_registers.c
)
target_link_libraries(test_sx127x sx127xlib)
add_test(NAME test_sx127x COMMAND test_sx127x)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_custom_target("coverage")
    get_filename_component(baseDir "${CMAKE_CURRENT_SOURCE_DIR}/.." REALPATH BASE_DIR)
    add_custom_command(TARGET "coverage" COMMAND gcov ${CMAKE_BINARY_DIR}/CMakeFiles/sx127xlib.dir/${baseDir}/src/*.c.o)
endif()