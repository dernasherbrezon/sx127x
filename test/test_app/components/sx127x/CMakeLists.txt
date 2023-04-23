set(srcs
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src/sx127x.c"
)
list(APPEND srcs "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src/sx127x_esp_spi.c")
idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../../include" REQUIRES "driver")
