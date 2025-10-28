idf_component_register(SRCS "wifi.c" "hello_world_main.c"
                       REQUIRES driver freertos esp_timer
                       INCLUDE_DIRS ".")