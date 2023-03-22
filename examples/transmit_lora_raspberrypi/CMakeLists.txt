cmake_minimum_required(VERSION 3.5)
project(transmit_lora_raspberrypi C)

set(CMAKE_C_STANDARD 99)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../ ${CMAKE_CURRENT_BINARY_DIR}/sx127x)

add_executable(transmit_lora_raspberrypi main.c)
target_link_libraries(transmit_lora_raspberrypi sx127x)