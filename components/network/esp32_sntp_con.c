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
    ESP_LOGI(TAG, "Aguardando Wi-Fi para iniciar SNTP...");
    if (!wifi_wait_connected()) {
        ESP_LOGE(TAG, "Wi-Fi não conectou após 15s, abortando SNTP");
        return;
    }

    ESP_LOGI(TAG, "Wi-Fi conectado! Iniciando SNTP...");

    setenv("TZ", "BRT3BRST,M10.3.0/0,M2.3.0/0", 1);
    tzset();

    esp_sntp_stop(); // força reinício do cliente SNTP, se já estiver rodando
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "a.ntp.br");
    esp_sntp_setservername(1, "b.ntp.br");
    esp_sntp_setservername(2, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    // Espera até sincronizar
    for (int i = 0; i < 40; ++i) {  // aumenta tempo de espera
        time_t now;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2016 - 1900)) {
            char buf[64];
            strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
            ESP_LOGI(TAG, "✅ Hora sincronizada com sucesso: %s", buf);
            return;
        }
        ESP_LOGW(TAG, "Aguardando sincronização SNTP... (%d)", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG, "⛔ SNTP não sincronizou dentro do tempo esperado!");
}
