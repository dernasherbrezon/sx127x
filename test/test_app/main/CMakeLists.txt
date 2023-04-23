set(app_sources "main.c" "sx127x_fixture.c")

idf_component_register(SRCS ${app_sources}
        REQUIRES unity driver sx127x freertos
        )