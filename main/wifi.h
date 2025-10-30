#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configurações de Wi-Fi
#define WIFI_SSID "Nilocas"
#define WIFI_PASS "nilocass"

// Funções exportadas
void wifi_init_sta(void);
void tcp_server_task(void *pvParameters);
