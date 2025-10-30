/*
Arquivo: esp32_sntp_con.c
Autor: Felipe Viel
Disciplina: Sistemas em Tempo Real
Engenharia de Computação - Univali
Descrição: Código que permite a ESP32 capturar a hora e data atual em servidor SNTP
Garanta que no CMakeList esteja a configuração abaixo:

idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash
                       INCLUDE_DIRS "")
*/

#include "esp32_sntp_con.h"
#include "tcp_udp_esp32_wifi_con.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>
extern time_t last_sync_time;
static const char *TAG = "TIME_SYNC";
extern time_t last_sync_time; 
void time_sync_notification_cb(struct timeval *tv) {
    time(&last_sync_time);
    ESP_LOGI("SNTP", "Hora sincronizada! last_sync_time = %ld", last_sync_time);
}

void time_sync_init(void)
{
    if (!wifi_wait_connected()) {
        ESP_LOGE(TAG, "Wi-Fi não conectou após 15s, abortando SNTP");
        return;
    }

    ESP_LOGI(TAG, "Wi-Fi conectado! Iniciando SNTP...");

    setenv("TZ", "BRT3", 1);
    tzset();

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_setservername(0, "a.st1.ntp.br");
    sntp_setservername(1, "b.st1.ntp.br");
    sntp_setservername(2, "pool.ntp.org");
    sntp_init();

    for (int i = 0; i < 20; ++i) {
        time_t now;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2016 - 1900)) {
            char buf[64];
            strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
            ESP_LOGI(TAG, "Hora sincronizada: %s", buf);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGW(TAG, "SNTP não sincronizou dentro do tempo esperado");
}
