/*
Arquivo: tcp_udp_esp32_wifi_con.c
Autor: Felipe Viel
Disciplina: Sistemas em Tempo Real
Engenharia de Computação - Univali
Descrição: Código que torna a ESP32 Cliente (TCP) ou Servidor (UDP) na troca de pacotes com um computador
Garanta que no CMakeList esteja a configuração abaixo:

idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash
                       INCLUDE_DIRS "")
*/
#include <string.h>
#include <stdio.h>
#include <stdint.h>        // garante que int32_t e afins existam

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"     // importante antes de qualquer coisa do lwIP

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "tcp_udp_esp32_wifi_con.h"
#include "esp32_sntp_con.h"


#define WIFI_SSID "fernando"
#define WIFI_PASS "fernando123"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TCP_PORT 6010
#define PC_IP   "192.168.15.5"
#define PC_PORT 3333

// Externas para acionar tasks principais
extern TaskHandle_t fsHandle;
extern TaskHandle_t navHandle;
extern QueueHandle_t navQueue;

static const char *TAG_TCP = "TCP_SERVER";

static const char *TAG = "NETWORK";
static EventGroupHandle_t s_wifi_event_group;

int tcp_client_sock = -1;


/* ============================================================
 *               HANDLERS DE EVENTOS WI-FI
 * ============================================================ */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Tentando reconectar ao Wi-Fi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi conectado e IP obtido.");
    }
}

/* ============================================================
 *                INICIALIZAÇÃO DO WI-FI
 * ============================================================ */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi inicializado (modo STA)");
}

/* ============================================================
 *             FUNÇÃO DE BLOQUEIO ATÉ CONEXÃO
 * ============================================================ */
bool wifi_wait_connected(void)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000)
    );

    return (bits & WIFI_CONNECTED_BIT);
}

/* ============================================================
 *                ENVIO UDP SIMPLES (para MONITOR_TASK)
 * ============================================================ */
esp_err_t send_udp_message(const char *msg, const char *ip, int port)
{
    if (!msg || !ip) return ESP_FAIL;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);

    if (inet_pton(AF_INET, ip, &dest.sin_addr.s_addr) != 1) {
        ESP_LOGE(TAG, "IP inválido: %s", ip);
        return ESP_FAIL;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Erro ao criar socket UDP");
        return ESP_FAIL;
    }

    ssize_t sent = sendto(sock, msg, strlen(msg), 0,
                          (struct sockaddr *)&dest, sizeof(dest));

    close(sock);
    if (sent < 0) {
        ESP_LOGE(TAG, "Erro ao enviar UDP para %s:%d", ip, port);
        return ESP_FAIL;
    }

    // sucesso
    return ESP_OK;
}

/* ============================================================
 *             TAREFAS TCP / UDP (inalteradas)
 * ============================================================ */

static void udp_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &dest.sin_addr.s_addr);

    while (1) {
        char payload[64];
        static int seq = 0;
        snprintf(payload, sizeof(payload),
                 "{\"seq\":%d,\"ax\":0.12,\"ay\":-0.03}", seq++);
        sendto(sock, payload, strlen(payload), 0,
               (struct sockaddr*)&dest, sizeof(dest));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void tcp_server_task(void *arg)
{
    int listen_fd, sock;
    struct sockaddr_in addr, source_addr;
    socklen_t addr_len = sizeof(source_addr);

    // Cria socket TCP
    listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG_TCP, "Erro criando socket TCP");
        vTaskDelete(NULL);
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG_TCP, "Erro no bind()");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_fd, 1) < 0) {
        ESP_LOGE(TAG_TCP, "Erro no listen()");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_TCP, "Servidor TCP ativo na porta %d", TCP_PORT);

    while (1) {
        sock = accept(listen_fd, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG_TCP, "Erro no accept()");
            continue;
        }

        ESP_LOGI(TAG_TCP, "Cliente conectado!");

        char rx_buffer[128];
        int len;

        while ((len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0)) > 0) {
            for (int i = 0; i < len; i++) {
                char c = rx_buffer[i];

                switch (c) {
                    case 'f':   // Aciona FS_TASK
                        if (fsHandle) {
                            xTaskNotifyGive(fsHandle);
                            ESP_LOGI(TAG_TCP, "FS_TASK acionada (comando 'f')");
                        }
                        break;

                    case 'n':   // Aciona NAV_PLAN
                        if (navQueue) {
                            int evt = 1;
                            xQueueSend(navQueue, &evt, 0);
                            ESP_LOGI(TAG_TCP, "NAV_PLAN acionada (comando 'n')");
                        }
                        break;

                    case 't':   // Teste — resposta simples
                        ESP_LOGI(TAG_TCP, "Ping recebido ('t')");
                        send(sock, "pong\n", 5, 0);
                        break;

                    default:
                        // Ignora bytes de controle do Telnet (ÿ, etc)
                        if (c >= 32 && c <= 126)
                            ESP_LOGW(TAG_TCP, "Comando desconhecido: %c", c);
                        break;
                }
            }
        }

        if (len <= 0) {
            ESP_LOGW(TAG_TCP, "Cliente desconectado");
        }
        shutdown(sock, 0);
        close(sock);
    }
}