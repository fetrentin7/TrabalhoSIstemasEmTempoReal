#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void wifi_init_sta(void);
void tcp_server_task(void *pvParameter);

#ifdef __cplusplus
}
#endif