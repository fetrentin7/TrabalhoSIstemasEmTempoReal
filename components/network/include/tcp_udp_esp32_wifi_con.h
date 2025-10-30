#pragma once
#include <stdbool.h> 
#include "esp_err.h"   
#ifdef __cplusplus
extern "C" {
#endif

void wifi_init_sta(void);
//void tcp_server_task(void *pvParameter);
bool wifi_wait_connected(void);
esp_err_t send_udp_message(const char *msg, const char *ip, int port);  
#ifdef __cplusplus
}
#endif