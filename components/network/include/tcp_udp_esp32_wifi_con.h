#pragma once
#include <stdbool.h> 
#ifdef __cplusplus
extern "C" {
#endif

void wifi_init_sta(void);
//void tcp_server_task(void *pvParameter);
bool wifi_wait_connected(void);

#ifdef __cplusplus
}
#endif