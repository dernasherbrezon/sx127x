set(srcs
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sx127x.c"
)
# When running from IDF build it as a component
if (IDF_TARGET)
    list(APPEND srcs "src/sx127x_esp_spi.c")
    idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "include" REQUIRES "driver")
else()
    list(APPEND srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/sx127x_linux_spi.c")
    add_library(sx127x STATIC ${srcs})
    target_include_directories(sx127x PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()
