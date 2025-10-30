idf_component_register(
    SRCS 
        "hello_world_main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos network
)