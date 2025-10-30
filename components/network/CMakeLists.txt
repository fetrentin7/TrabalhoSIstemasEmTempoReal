idf_component_register(
    SRCS "tcp_udp_esp32_wifi_con.c" "esp32_sntp_con.c" "monitor_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash lwip freertos
)
