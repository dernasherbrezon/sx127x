set(app_sources "main.c" "uart_at.c" "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/sx127x_at.c")

idf_component_register(SRCS ${app_sources}
        REQUIRES driver sx127x freertos
        )