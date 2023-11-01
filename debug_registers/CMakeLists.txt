cmake_minimum_required(VERSION 3.5)
project(debug_registers C)

set(CMAKE_C_STANDARD 99)

add_executable(debug_registers
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)
