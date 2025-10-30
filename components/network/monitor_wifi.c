#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <stdio.h>
#include "monitor_wifi.h"

// ====== Configurações de rede ======
#define PC_IP   "192.168.15.5"   // IP do seu computador
#define PC_PORT_UDP 6010         // porta UDP
#define PC_PORT_TCP 6011         // porta TCP para comandos

// ====== Variáveis globais externas (vindas do monitor principal) ======
extern bool sim_running;
extern bool sim_just_started;
extern bool sim_finished;

extern uint32_t fus_imu_deadline_misses;
extern uint32_t ctrl_deadline_misses;
extern uint32_t fs_deadline_misses;
extern uint32_t nav_soft_misses;
extern uint64_t max_fs_latency_us;
extern int64_t  max_jitter_us;

// ====== LOG ======
static const char *TAG = "MONITOR_WIFI";

void MONITOR_WIFI_TASK(void *pvParameters)
{
    ESP_LOGI(TAG, "Task start");

    // Dá um tempo após Wi-Fi ficar ON (IP obtido)
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ===== UDP destino =====
    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(PC_PORT_UDP);
    if (inet_pton(AF_INET, PC_IP, &dest_addr.sin_addr.s_addr) != 1) {
        ESP_LOGE(TAG, "inet_pton falhou p/ %s", PC_IP);
        vTaskDelete(NULL); return;
    }

    // ===== Socket UDP =====
    ESP_LOGI(TAG, "Criando socket UDP...");
    int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Falha socket UDP: errno=%d", errno);
        vTaskDelete(NULL); return;
    }

    // ===== Socket TCP (comandos) =====
    ESP_LOGI(TAG, "Criando socket TCP...");
    int tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (tcp_sock < 0) {
        ESP_LOGE(TAG, "Falha socket TCP: errno=%d", errno);
        close(udp_sock);
        vTaskDelete(NULL); return;
    }

    // Non-blocking no socket de escuta
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family      = AF_INET;
    listen_addr.sin_port        = htons(PC_PORT_TCP);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    ESP_LOGI(TAG, "Bind TCP porta %d...", PC_PORT_TCP);
    if (bind(tcp_sock, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG, "bind TCP falhou: errno=%d", errno);
        close(udp_sock); close(tcp_sock);
        vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "Listen TCP...");
    if (listen(tcp_sock, 1) < 0) {
        ESP_LOGE(TAG, "listen TCP falhou: errno=%d", errno);
        close(udp_sock); close(tcp_sock);
        vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "Monitor Wi-Fi ativo (UDP %d / TCP %d)", PC_PORT_UDP, PC_PORT_TCP);

    TickType_t last_wake_time = xTaskGetTickCount();
    char udp_msg[256];
    char rx_buf[128];

    while (1) {
        // -------- envio UDP ----------
        int n = snprintf(udp_msg, sizeof(udp_msg),
            "{\"fus_misses\":%lu,\"ctrl_misses\":%lu,\"fs_misses\":%lu,\"nav_soft\":%lu,"
            "\"fs_lat_us\":%llu,\"fus_jitter_us\":%lld,\"sim_running\":%d}",
            fus_imu_deadline_misses, ctrl_deadline_misses, fs_deadline_misses,
            nav_soft_misses, (unsigned long long)max_fs_latency_us,
            (long long)max_jitter_us, sim_running);

        if (sendto(udp_sock, udp_msg, n, 0,
                   (struct sockaddr*)&dest_addr, sizeof(dest_addr)) < 0) {
            ESP_LOGW(TAG, "sendto UDP erro: errno=%d", errno);
        }

        // -------- aceita comando TCP (non-blocking) ----------
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int client = accept(tcp_sock, (struct sockaddr*)&source_addr, &addr_len);
        if (client > 0) {
            int len = recv(client, rx_buf, sizeof(rx_buf)-1, 0);
            if (len > 0) {
                rx_buf[len] = 0;
                ESP_LOGI(TAG, "Comando: %s", rx_buf);

                if (strstr(rx_buf, "START")) {
                    sim_running = true;
                    sim_just_started = true;
                    sim_finished = false;
                } else if (strstr(rx_buf, "STOP")) {
                    sim_running = false;
                } else if (strstr(rx_buf, "RESET")) {
                    sim_just_started = true;
                    sim_finished = false;
                    fus_imu_deadline_misses = ctrl_deadline_misses =
                    fs_deadline_misses = nav_soft_misses = 0;
                }
            }
            shutdown(client, 0);
            close(client);
        } else {
            // client == -1: se for EWOULDBLOCK/EAGAIN, é esperado (non-blocking)
            if (errno != EWOULDBLOCK && errno != EAGAIN) {
                // loga, mas mantém a task viva
                // ESP_LOGD(TAG, "accept: errno=%d", errno);
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }

    // Nunca chega aqui
    close(udp_sock);
    close(tcp_sock);
    vTaskDelete(NULL);
}