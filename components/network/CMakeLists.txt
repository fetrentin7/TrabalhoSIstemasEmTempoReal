idf_component_register(
    SRCS "esp32_sntp_con.c"
         "tcp_udp_esp32_wifi_con.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash lwip freertos
)