cmake_minimum_required(VERSION 3.5)
project(sx127x_test C)

set(CMAKE_C_STANDARD 99)

add_library(sx127xlib
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sx127x.c
)

find_package(PkgConfig REQUIRED)

enable_testing()

pkg_check_modules(PC_CHECK REQUIRED check)
include_directories(${PC_CHECK_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
link_directories(${PC_CHECK_LIBRARY_DIRS})

add_executable(test_sx127x 
    ${CMAKE_CURRENT_SOURCE_DIR}/test_sx127x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sx127x_mock_spi.c
)
target_link_libraries(test_sx127x sx127xlib ${PC_CHECK_LIBRARIES})
add_test(NAME test_sx127x COMMAND test_sx127x)
