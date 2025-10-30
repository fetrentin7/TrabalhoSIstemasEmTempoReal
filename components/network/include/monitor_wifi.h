#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Task que envia status via UDP e recebe comandos via TCP.
 * Deve ser criada após o Wi-Fi obter IP.
 */
void MONITOR_WIFI_TASK(void *pvParameters);

#ifdef __cplusplus
}
#endif
