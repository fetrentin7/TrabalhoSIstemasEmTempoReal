#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/touch_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "sdkconfig.h"
#include "tcp_udp_esp32_wifi_con.h"
#include "esp32_sntp_con.h"
#include "nvs_flash.h"
#include <time.h>
#include <sys/time.h>
#include "lwip/sockets.h"


// ---------------- CONFIGURAÇÕES ----------------
#define TOUCH_NAV       9
#define TOUCH_TLM       3
#define TOUCH_FS        4

// ---------------- POLÍTICAS DE PRIORIDADE ----------------
#define POLICY_RM     1
#define POLICY_DM     2
#define POLICY_CUSTOM 3
#define SELECTED_POLICY POLICY_CUSTOM // <-- altere aqui para alternar entre políticas

// ---------------- PROTOCOLO DE COMUNICAÇÃO ----------------
#define PROTO_UDP   1
#define PROTO_TCP   2
#define SELECTED_PROTO PROTO_TCP   // <-- altere aqui para alternar entre UDP e TCP

#define FUS_IMU_PERIOD_MS 5

#define DEADLINE_FUS_IMU_US   (FUS_IMU_PERIOD_MS * 1000)
#define DEADLINE_CTRL_ATT_US  5000
#define DEADLINE_FS_TASK_US   10000
#define DEADLINE_NAV_PLAN_US  20000

#define REPORT_PC_IP     "192.168.15.17"   // <-- ajuste para o IP do seu PC
#define REPORT_UDP_PORT  3333
#if SELECTED_PROTO == PROTO_UDP
    #define REPORT_PROTO_STR "UDP" // campo informativo no JSON
#elif SELECTED_PROTO == PROTO_TCP
    #define REPORT_PROTO_STR "TCP" // campo informativo no JSON
#else
    #define REPORT_PROTO_STR "UNKNOWN"
#endif             // campo informativo no JSON

// Seq do relatório + referência temporal
static volatile uint64_t g_report_seq = 0;
static int64_t g_last_report_time_us = 0;

// último sync para a TIME_TASK exibir
time_t last_sync_time = 0;

// ---------------- VARIÁVEIS GLOBAIS ----------------
// --- Estruturas de Estatísticas e CPU Time ---
typedef struct {
    uint32_t fus_samples, fus_misses;
    uint32_t ctrl_runs, ctrl_misses;
    uint32_t nav_events, nav_soft_misses;
    uint32_t fs_events, fs_misses;
    int64_t fus_lat_total_us;
    uint32_t fus_lat_count;
    int64_t ctrl_lat_total_us;
    uint32_t ctrl_lat_count;
    int64_t nav_lat_total_us;
    uint32_t nav_lat_count;
    int64_t fs_lat_total_us;
    uint32_t fs_lat_count;
} report_stats_t;

typedef struct {
    int64_t fus_time_us;
    int64_t ctrl_time_us;
    int64_t nav_time_us;
    int64_t fs_time_us;
    int64_t monitor_time_us;
} report_cpu_time_t;

// --- Estrutura para Monitor de Firm Real-Time ---
typedef struct {
    uint8_t window_size;
    uint8_t m_required;
    uint8_t idx;
    bool *history;
    uint32_t total_exec;
    uint32_t met_in_window;
} mk_firm_t;

// --- Instâncias globais ---
static report_stats_t stats = {0};
static report_cpu_time_t cpu_time = {0};
static SemaphoreHandle_t mutexStats;
static mk_firm_t mk_ctrl, mk_nav;
QueueHandle_t navQueue = NULL;

// --- Handles das tasks ---
TaskHandle_t fusHandle, ctrlHandle, navHandle, fsHandle, monitorHandle;

// --- Variáveis de controle ---
static volatile uint64_t fs_trigger_time_us = 0;
static volatile uint64_t last_nav_event_time_us = 0;
static volatile uint64_t last_tlm_event_time_us = 0;

static uint64_t ctrl_wcrt = 0;
static uint64_t nav_wcrt = 0;

static uint32_t last_pad_status = 0;
QueueHandle_t navQueue;

extern int tcp_client_sock;


// ---------------- FUNÇÃO DE CARGA ----------------
void cargaCPU_us(uint32_t micros) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < micros) {
        __asm__ volatile ("nop");
    }
}

// ---------------- TASKS ----------------
void FUS_IMU(void *pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FUS_IMU_PERIOD_MS));
        uint64_t t_start = esp_timer_get_time();

        // --- Execução simulada ---
        cargaCPU_us(500);

        uint64_t t_end = esp_timer_get_time();
        uint64_t exec_time_us = t_end - t_start;

        if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
            stats.fus_samples++;
            if (exec_time_us > DEADLINE_FUS_IMU_US) stats.fus_misses++;
            cpu_time.fus_time_us += exec_time_us;
            stats.fus_lat_total_us += exec_time_us;
            stats.fus_lat_count++;
            xSemaphoreGive(mutexStats);
        }

        xTaskNotifyGive(ctrlHandle);
    }
}

void CTRL_ATT(void *pvParameter) {
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
            uint64_t t_start = esp_timer_get_time();
            cargaCPU_us(500);
            uint64_t t_end = esp_timer_get_time();
            uint64_t exec_time_us = t_end - t_start;
            
            bool met_deadline = (exec_time_us <= DEADLINE_CTRL_ATT_US);
            mk_ctrl.history[mk_ctrl.idx] = met_deadline;
            mk_ctrl.idx = (mk_ctrl.idx + 1) % mk_ctrl.window_size;
            mk_ctrl.total_exec++;

            uint8_t count = 0;
            for (int i = 0; i < mk_ctrl.window_size; i++)
                if (mk_ctrl.history[i]) count++;

            mk_ctrl.met_in_window = count;

            if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
                static uint64_t ctrl_wcrt = 0;
                if (exec_time_us > ctrl_wcrt) ctrl_wcrt = exec_time_us;
                stats.ctrl_runs++;
                if (exec_time_us > DEADLINE_CTRL_ATT_US) stats.ctrl_misses++;
                cpu_time.ctrl_time_us += exec_time_us;
                stats.ctrl_lat_total_us += exec_time_us;
                stats.ctrl_lat_count++;
                xSemaphoreGive(mutexStats);
            }
        }
    }
}

void NAV_PLAN(void *pvParameter) {
    int evt;
    static uint64_t nav_wcrt = 0;  // mantém o pior tempo já visto

    while (1) {
        if (xQueueReceive((QueueHandle_t)pvParameter, &evt, portMAX_DELAY)) {
            uint64_t t_start = esp_timer_get_time();

            // --- Simulação de carga ---
            cargaCPU_us(2500);

            uint64_t t_end = esp_timer_get_time();
            uint64_t exec_time_us = t_end - t_start;

            // --- (m,k)-firm ---
            bool met_deadline = (exec_time_us <= DEADLINE_NAV_PLAN_US);
            mk_nav.history[mk_nav.idx] = met_deadline;
            mk_nav.idx = (mk_nav.idx + 1) % mk_nav.window_size;
            mk_nav.total_exec++;

            // Conta quantas execuções da janela atual cumpriram o deadline
            uint8_t count = 0;
            for (int i = 0; i < mk_nav.window_size; i++) {
                if (mk_nav.history[i]) count++;
            }
            mk_nav.met_in_window = count;

            // --- Atualiza estatísticas globais ---
            if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
                stats.nav_events++;
                if (!met_deadline) stats.nav_soft_misses++;
                cpu_time.nav_time_us += exec_time_us;
                stats.nav_lat_total_us += exec_time_us;
                stats.nav_lat_count++;

                // Atualiza WCRT se este tempo foi o pior até agora
                if (exec_time_us > nav_wcrt) {
                    nav_wcrt = exec_time_us;
                }

                xSemaphoreGive(mutexStats);
            }
        }
    }
}


void FS_TASK(void *pvParameter) {
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
            uint64_t t_start = esp_timer_get_time();
            cargaCPU_us(500);
            uint64_t t_end = esp_timer_get_time();
            uint64_t exec_time_us = t_end - t_start;

            if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
                stats.fs_events++;
                if (exec_time_us > DEADLINE_FS_TASK_US) stats.fs_misses++;
                cpu_time.fs_time_us += exec_time_us;
                stats.fs_lat_total_us += exec_time_us;
                stats.fs_lat_count++;
                xSemaphoreGive(mutexStats);
            }
        }
    }
}

void MONITOR_TASK(void *pvParameters) {
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5000); // 5 s

    // marcos temporais
    if (g_last_report_time_us == 0) {
        g_last_report_time_us = esp_timer_get_time();
    }

    while (1) {
        vTaskDelayUntil(&next, period);

        // --- começo da janela do relatório (para medir CPU % da janela) ---
        int64_t t_start = esp_timer_get_time();

        // ====== Cálculo das métricas (como você já fazia) ======
        int64_t avg_fus=0, avg_ctrl=0, avg_nav=0, avg_fs=0;
        float cpu_pct = 0.0f;
        uint32_t fus_samples=0, fus_misses=0;
        uint32_t ctrl_runs=0, ctrl_misses=0;
        uint32_t nav_events=0, nav_soft_misses=0;
        uint32_t fs_events=0, fs_misses=0;

        if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
            // médias
            avg_fus  = stats.fus_lat_count ? stats.fus_lat_total_us / stats.fus_lat_count : 0;
            avg_ctrl = stats.ctrl_lat_count ? stats.ctrl_lat_total_us / stats.ctrl_lat_count : 0;
            avg_nav  = stats.nav_lat_count ? stats.nav_lat_total_us  / stats.nav_lat_count  : 0;
            avg_fs   = stats.fs_lat_count  ? stats.fs_lat_total_us   / stats.fs_lat_count   : 0;

            // snapshot dos contadores (pra imprimir/enviar)
            fus_samples      = stats.fus_samples;
            fus_misses       = stats.fus_misses;
            ctrl_runs        = stats.ctrl_runs;
            ctrl_misses      = stats.ctrl_misses;
            nav_events       = stats.nav_events;
            nav_soft_misses  = stats.nav_soft_misses;
            fs_events        = stats.fs_events;
            fs_misses        = stats.fs_misses;

            // uso de CPU da janela (com base no período)
            float cpu_total_us = (float)(cpu_time.fus_time_us + cpu_time.ctrl_time_us +
                                         cpu_time.nav_time_us + cpu_time.fs_time_us);
            cpu_pct = cpu_total_us / (period * 1000.0f) * 100.0f;

            // imprime console (igual antes)
            printf("\n=== RELATÓRIO PERIÓDICO ===\n");
            printf("FUS_IMU: samples=%lu  misses=%lu  avg_exec=%lld us\n",
                   fus_samples, fus_misses, (long long)avg_fus);
            printf("CTRL_ATT: runs=%lu  misses=%lu  avg_exec=%lld us\n",
                   ctrl_runs, ctrl_misses, (long long)avg_ctrl);
            printf("NAV_PLAN: evts=%lu  soft_misses=%lu  avg_exec=%lld us\n",
                   nav_events, nav_soft_misses, (long long)avg_nav);
            printf("FS_TASK: evts=%lu  misses=%lu  avg_exec=%lld us\n",
                   fs_events, fs_misses, (long long)avg_fs);
            printf("CTRL_ATT: (m,k)=(%u,%u) met_in_window=%lu  WCRT=%llu us\n",
                    mk_ctrl.m_required, mk_ctrl.window_size,
                    (unsigned long)mk_ctrl.met_in_window,
                    (unsigned long long)ctrl_wcrt);
            printf("NAV_PLAN: (m,k)=(%u,%u) met_in_window=%lu  WCRT=%llu us\n",
                    mk_nav.m_required, mk_nav.window_size,
                    (unsigned long)mk_nav.met_in_window,
                    (unsigned long long)nav_wcrt);
            printf("=============================\n\n");

            // ====== MONITOR: zera acumuladores de latência e cpu_time para PRÓXIMA janela ======
            stats.fus_lat_total_us = stats.ctrl_lat_total_us =
            stats.nav_lat_total_us = stats.fs_lat_total_us = 0;
            stats.fus_lat_count = stats.ctrl_lat_count =
            stats.nav_lat_count = stats.fs_lat_count = 0;

            cpu_time.fus_time_us = cpu_time.ctrl_time_us =
            cpu_time.nav_time_us = cpu_time.fs_time_us = 0;

            xSemaphoreGive(mutexStats);
        }

        // ====== Construção do JSON + envio por UDP ======
        // timestamps SNTP + esp_timer (micro) + seq + tamanho do pacote
        struct timeval tv;
        gettimeofday(&tv, NULL); // usa SNTP se já sincronizado
        int64_t esp_now_us = esp_timer_get_time();
        int64_t period_us  = esp_now_us - g_last_report_time_us;
        g_last_report_time_us = esp_now_us;

        g_report_seq++;

        char json[512];
        // 1ª passagem com pkt_size=0 só pra calcular o tamanho
        int len = snprintf(json, sizeof(json),
            "{"
            "\"type\":\"drone_report\","
            "\"seq\":%llu,"
            "\"protocol\":\"%s\","
            "\"pkt_size\":%d,"
            "\"sntp_time_s\":%ld,"
            "\"sntp_time_us\":%ld,"
            "\"esp_time_us\":%lld,"
            "\"period_us\":%lld,"
            "\"fus\":{\"samples\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"ctrl\":{\"runs\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"nav\":{\"events\":%lu,\"soft_misses\":%lu,\"avg_us\":%lld},"
            "\"fs\":{\"events\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"cpu_usage_pct\":%.2f,"
            "\"mk_ctrl_met\":%lu,\"mk_nav_met\":%lu,"
            "\"ctrl_wcrt_us\":%llu,\"nav_wcrt_us\":%llu"
            "}",
            (unsigned long long)g_report_seq,
            REPORT_PROTO_STR,
            0,
            (long)tv.tv_sec, (long)tv.tv_usec,
            (long long)esp_now_us,
            (long long)period_us,
            fus_samples, fus_misses, (long long)avg_fus,
            ctrl_runs, ctrl_misses, (long long)avg_ctrl,
            nav_events, nav_soft_misses, (long long)avg_nav,
            fs_events, fs_misses, (long long)avg_fs,
            cpu_pct,
            (unsigned long)mk_ctrl.met_in_window,
            (unsigned long)mk_nav.met_in_window,
            (unsigned long long)ctrl_wcrt,
            (unsigned long long)nav_wcrt
        );

        // 2ª passagem já com o tamanho correto
        len = snprintf(json, sizeof(json),
            "{"
            "\"type\":\"drone_report\","
            "\"seq\":%llu,"
            "\"protocol\":\"%s\","
            "\"pkt_size\":%d,"
            "\"sntp_time_s\":%ld,"
            "\"sntp_time_us\":%ld,"
            "\"esp_time_us\":%lld,"
            "\"period_us\":%lld,"
            "\"fus\":{\"samples\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"ctrl\":{\"runs\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"nav\":{\"events\":%lu,\"soft_misses\":%lu,\"avg_us\":%lld},"
            "\"fs\":{\"events\":%lu,\"misses\":%lu,\"avg_us\":%lld},"
            "\"cpu_usage_pct\":%.2f"
            "}",
            (unsigned long long)g_report_seq,
            REPORT_PROTO_STR,
            len,
            (long)tv.tv_sec, (long)tv.tv_usec,
            (long long)esp_now_us,
            (long long)period_us,
            fus_samples, fus_misses, (long long)avg_fus,
            ctrl_runs,   ctrl_misses, (long long)avg_ctrl,
            nav_events,  nav_soft_misses, (long long)avg_nav,
            fs_events,   fs_misses, (long long)avg_fs,
            cpu_pct
        );

        #if SELECTED_PROTO == PROTO_UDP
            send_udp_message(json, REPORT_PC_IP, REPORT_UDP_PORT);
            printf("[UDP] enviado para %s:%d | %s\n", REPORT_PC_IP, REPORT_UDP_PORT, json);
        #elif SELECTED_PROTO == PROTO_TCP
            if (tcp_client_sock > 0) {
                send(tcp_client_sock, json, strlen(json), 0);
                printf("[TCP] enviado para cliente conectado | %s\n", json);
            } else {
                printf("[TCP] Nenhum cliente conectado, pulando envio.\n");
            }
        #endif


        // --- fecha a janela do relatório e contabiliza o tempo da PRÓPRIA monitor_task ---
        int64_t t_end = esp_timer_get_time();
        if (xSemaphoreTake(mutexStats, portMAX_DELAY) == pdTRUE) {
            cpu_time.monitor_time_us += (t_end - t_start);
            xSemaphoreGive(mutexStats);
        }
    }
}


void TIME_TASK(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    time_t now; struct tm ti;

    while (1) {
        time(&now);
        localtime_r(&now, &ti);
        double since_sync = difftime(now, last_sync_time);

        char buf[64];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &ti);
        printf("[TIME_TASK] %s | %.0fs desde último sync SNTP\n", buf, since_sync);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}



// ---------------- INTERRUPÇÃO TOUCH ----------------
static void tp_isr(void *arg) {
    uint32_t status = touch_pad_get_status();
    touch_pad_clear_status();
    BaseType_t hp = pdFALSE;
    uint32_t rising = status & ~last_pad_status;

    if ((rising >> TOUCH_NAV) & 0x01) {
        last_nav_event_time_us = esp_timer_get_time();
        int e = TOUCH_NAV;
        xQueueSendFromISR(navQueue, &e, &hp);
    }
    if ((rising >> TOUCH_TLM) & 0x01) {
        last_tlm_event_time_us = esp_timer_get_time();
        int e = TOUCH_TLM;
        xQueueSendFromISR(navQueue, &e, &hp);
    }
    if ((rising >> TOUCH_FS) & 0x01) {
        fs_trigger_time_us = esp_timer_get_time();
        vTaskNotifyGiveFromISR(fsHandle, &hp);
    }

    last_pad_status = status;
    if (hp) portYIELD_FROM_ISR();
}


// ---------------- APP MAIN ----------------
void app_main(void) {
    // NVS + Wi-Fi + SNTP
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    printf("Conectando ao Wi-Fi...\n");
    if (wifi_wait_connected()) {
        printf("Wi-Fi conectado! Sincronizando SNTP...\n");
        time_sync_init(); // deve atualizar last_sync_time internamente (ou faça callback)
    } else {
        printf("Falha ao conectar ao Wi-Fi.\n");
    }
    
    #if SELECTED_PROTO == PROTO_TCP
        xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 2, NULL);
    #endif

    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));

    ESP_ERROR_CHECK(touch_pad_config(TOUCH_NAV, 0));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_TLM, 0));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_FS, 0));

    ESP_ERROR_CHECK(touch_pad_filter_start(10));
    vTaskDelay(pdMS_TO_TICKS(200));

    uint16_t val;
    touch_pad_read_filtered(TOUCH_NAV, &val);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_NAV, (uint32_t)(val * 0.8f)));
    touch_pad_read_filtered(TOUCH_TLM, &val);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_TLM, (uint32_t)(val * 0.8f)));
    touch_pad_read_filtered(TOUCH_FS, &val);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_FS, (uint32_t)(val * 0.8f)));

    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();
    last_pad_status = pad_intr;

    ESP_ERROR_CHECK(touch_pad_isr_register(tp_isr, NULL));
    ESP_ERROR_CHECK(touch_pad_intr_enable());

    navQueue = xQueueCreate(10, sizeof(int));
    if (navQueue == NULL) {
        ESP_LOGE("MAIN", "Falha ao criar navQueue!");
    }    
    
    mutexStats = xSemaphoreCreateMutex();


    #if SELECTED_POLICY == POLICY_RM
        xTaskCreate(FUS_IMU, "fus_imu", 4096, NULL, 6, &fusHandle);
        xTaskCreate(CTRL_ATT, "ctrl_att", 4096, NULL, 5, &ctrlHandle);
        xTaskCreate(FS_TASK, "fs_task", 4096, NULL, 4, &fsHandle);
        xTaskCreate(NAV_PLAN, "nav_plan", 4096, (void*)navQueue, 3, &navHandle);
        xTaskCreate(TIME_TASK, "time_task", 4096, NULL, 2, NULL);
        xTaskCreate(MONITOR_TASK, "monitor_task", 4096, NULL, 1, &monitorHandle);
    #elif SELECTED_POLICY == POLICY_DM
        xTaskCreate(FUS_IMU, "fus_imu", 4096, NULL, 6, &fusHandle);
        xTaskCreate(CTRL_ATT, "ctrl_att", 4096, NULL, 5, &ctrlHandle);
        xTaskCreate(FS_TASK, "fs_task", 4096, NULL, 4, &fsHandle);
        xTaskCreate(NAV_PLAN, "nav_plan", 4096, (void*)navQueue, 3, &navHandle);
        xTaskCreate(TIME_TASK, "time_task", 4096, NULL, 2, NULL);
        xTaskCreate(MONITOR_TASK, "monitor_task", 4096, NULL, 1, &monitorHandle);
    #elif SELECTED_POLICY == POLICY_CUSTOM
        xTaskCreate(FS_TASK, "fs_task", 4096, NULL, 6, &fsHandle);
        xTaskCreate(FUS_IMU, "fus_imu", 4096, NULL, 5, &fusHandle);
        xTaskCreate(CTRL_ATT, "ctrl_att", 4096, NULL, 4, &ctrlHandle);
        xTaskCreate(MONITOR_TASK, "monitor_task", 4096, NULL, 3, &monitorHandle);
        xTaskCreate(TIME_TASK, "time_task", 4096, NULL, 2, NULL);
        xTaskCreate(NAV_PLAN, "nav_plan", 4096, (void*)navQueue, 1, &navHandle);
    #endif

    mk_ctrl.window_size = 10;
    mk_ctrl.m_required = 9;
    mk_ctrl.history = calloc(mk_ctrl.window_size, sizeof(bool));

    mk_nav.window_size = 10;
    mk_nav.m_required = 9;
    mk_nav.history = calloc(mk_nav.window_size, sizeof(bool));

    printf("\nSistema inicializado. Iniciando simulação automaticamente...\n");
    printf("Coletando métricas de tempo real a cada 5 segundos.\n");
}