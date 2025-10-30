#include "tcp_udp_esp32_wifi_con.h"
#include "esp32_sntp_con.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <time.h>
#include "nvs_flash.h"  

static const char *TAG = "APP_MAIN";
static uint32_t time_cycles = 0;
time_t last_sync_time = 0;

void TIME_TASK(void *pvParameters)
{
    ESP_LOGI("TIME_TASK", "Task iniciada com sucesso");
    TickType_t last_wake_time = xTaskGetTickCount();
    time_t now;
    struct tm timeinfo;

    while (1) {
        time(&now);
        localtime_r(&now, &timeinfo);
        time_cycles++;

        double seconds_since_sync = difftime(now, last_sync_time);
        char buf[64];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
        ESP_LOGI("TIME_TASK", "Hora: %s | Ciclos: %lu | Segundos desde sync: %.0f",
                 buf, time_cycles, seconds_since_sync);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    ESP_LOGI(TAG, "Aguardando Wi-Fi conectar...");
    if (wifi_wait_connected()) {
        ESP_LOGI(TAG, "Wi-Fi conectado, iniciando SNTP...");
        time_sync_init();
    } else {
        ESP_LOGE(TAG, "Wi-Fi não conectou a tempo!");
    }

    if (xTaskCreate(TIME_TASK, "time_task", 4096, NULL, 3, NULL) == pdPASS)
        ESP_LOGI(TAG, "TIME_TASK criada com sucesso");
    else
        ESP_LOGE(TAG, "Falha ao criar TIME_TASK");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}