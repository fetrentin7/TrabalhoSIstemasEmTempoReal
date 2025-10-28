#include "wifi.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

#define PORT 3333
static const char *TAG = "WiFi_TCP";

// Handles externos (referenciados do main)
extern TaskHandle_t fsHandle;
extern QueueHandle_t navQueue;

// === Função Wi-Fi ===
void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Conectando ao Wi-Fi...");
    esp_wifi_connect();
    vTaskDelay(pdMS_TO_TICKS(5000));
}

// === Servidor TCP ===
void tcp_server_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 1);

    ESP_LOGI(TAG, "Servidor TCP escutando na porta %d", PORT);

    while (1) {
        struct sockaddr_in6 source_addr;
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Cliente conectado: %s", addr_str);

        send(sock, "Conexao estabelecida!\n", 23, 0);

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len <= 0) break;

            rx_buffer[len] = 0;
            ESP_LOGI(TAG, "Recebido: %s", rx_buffer);

            if (strncmp(rx_buffer, "FS", 2) == 0) {
                vTaskNotifyGive(fsHandle);
                send(sock, "Fail-Safe ativado\n", 18, 0);
            } else if (strncmp(rx_buffer, "NAV", 3) == 0) {
                int event = 2; // TOUCH_NAV
                xQueueSend(navQueue, &event, 0);
                send(sock, "Navegacao acionada\n", 20, 0);
            } else if (strncmp(rx_buffer, "PING", 4) == 0) {
                send(sock, "ESP32 vivo\n", 11, 0);
            }
        }

        ESP_LOGW(TAG, "Cliente desconectado");
        close(sock);
    }

    vTaskDelete(NULL);
}
