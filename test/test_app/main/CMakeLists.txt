set(app_sources "main.c" "test_lora.c")

idf_component_register(SRCS ${app_sources}
        REQUIRES unity driver
        WHOLE_ARCHIVE)