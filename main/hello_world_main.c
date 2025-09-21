#include <stdio.h>
#include <stdlib.h> // Necessário para a função llabs (valor absoluto para long long)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TOUCH_NAV   2  // Touch B -> Navegação
#define TOUCH_TLM   3  // Touch C -> Telemetria
#define TOUCH_FS    4  // Touch D -> Fail-Safe
#define TOUCH_DIST  5  // Touch A opcional -> Distúrbio

// Período da task de controle em milissegundos
#define FUS_IMU_PERIOD_MS 5

// Handles das tasks
TaskHandle_t fusHandle, ctrlHandle, navHandle, fsHandle, monitorHandle;

// --- INÍCIO: Variáveis para Instrumentação de Tempo Real ---
//  'volatile' para garantir que o compilador não otimize o acesso
// a estas variáveis, pois elas são modificadas em tasks e ISRs diferentes.
volatile uint32_t fus_imu_deadline_misses = 0;
volatile int64_t max_jitter_us = 0;
volatile uint64_t max_fs_latency_us = 0;

// Variáveis de apoio para os cálculos
static int64_t last_fus_imu_activation_time = -1;
static volatile uint64_t fs_trigger_time_us = 0;
// --- FIM: Variáveis de Instrumentação ---

// ---------------------- TASKS -------------------------

void FUS_IMU(void *pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        // vTaskDelayUntil garante a periodicidade, executando esta linha a cada 5ms.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FUS_IMU_PERIOD_MS));
        
        uint64_t start = esp_timer_get_time();

        // --- INÍCIO: INSTRUMENTAÇÃO (Jitter) ---
        // Jitter é a variação no tempo de ativação da task.
        if (last_fus_imu_activation_time != -1) {
            int64_t period_us = start - last_fus_imu_activation_time;
            // Calcula o desvio do período esperado (5000 µs)
            int64_t jitter_us = llabs(period_us - (FUS_IMU_PERIOD_MS * 1000));
            if (jitter_us > max_jitter_us) {
                max_jitter_us = jitter_us;
            }
        }
        last_fus_imu_activation_time = start;
        // --- FIM: INSTRUMENTAÇÃO ---

        // Simula fusão sensorial (printf removido para não afetar a medição de tempo)
        // printf("[FUS_IMU] Nova amostra inercial\n");

        // Notifica controle de atitude
        xTaskNotifyGive(ctrlHandle);

        uint64_t end = esp_timer_get_time();
        uint64_t exec_time_us = end - start;

        // --- INÍCIO: INSTRUMENTAÇÃO (Miss de Deadline) ---
        // Se o tempo de execução exceder o período, contamos como um "miss".
        if (exec_time_us > (FUS_IMU_PERIOD_MS * 1000)) {
            fus_imu_deadline_misses++;
        }
        // --- FIM: INSTRUMENTAÇÃO ---
    }
}

void CTRL_ATT(void *pvParameter) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // espera FUS_IMU
        // Simula controle de atitude (printf removido)
    }
}

void NAV_PLAN(void *pvParameter) {
    int evt;
    while(1) {
        if (xQueueReceive((QueueHandle_t)pvParameter, &evt, portMAX_DELAY)) {
            if (evt == TOUCH_NAV)
                printf("[NAV_PLAN] Novo waypoint recebido\n");
            else if (evt == TOUCH_TLM)
                printf("[NAV_PLAN] Enviando telemetria\n");
        }
    }
}

void FS_TASK(void *pvParameter) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // ativada por ISR
        
        // --- INÍCIO: INSTRUMENTAÇÃO (Latência) ---
    
        uint64_t fs_start_time_us = esp_timer_get_time();
        
        // Calcula a latência se o tempo do gatilho foi registrado.
        if (fs_trigger_time_us > 0) {
            uint64_t latency_us = fs_start_time_us - fs_trigger_time_us;
            if (latency_us > max_fs_latency_us) {
                max_fs_latency_us = latency_us;
            }
        }
        // --- FIM: INSTRUMENTAÇÃO ---

        printf("[FS_TASK] FAIL-SAFE ACIONADO! Latencia Max: %llu us\n", max_fs_latency_us);
    }
}

// --- INÍCIO: NOVA TASK DE MONITORAMENTO ---
// Task de baixa prioridade para imprimir as métricas periodicamente.
void MONITOR_TASK(void* pvParameters) {
    while(1) {
        printf("\n--- METRICAS DE TEMPO REAL ---\n");
        printf("Deadline Misses (FUS_IMU): %lu\n", fus_imu_deadline_misses);
        printf("Max Jitter (FUS_IMU):      %lld us\n", max_jitter_us);
        printf("Max Latencia (Fail-Safe):  %llu us\n", max_fs_latency_us);
        printf("------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(2000)); // Imprime a cada 2 segundos
    }
}
// --- FIM: NOVA TASK ---


// ---------------------- TOUCH ISR -------------------------

static QueueHandle_t navQueue;

static void tp_isr(void *arg) {
    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Em vez de um loop, verificamos diretamente os pads que nos interessam
    if ((pad_intr >> TOUCH_NAV) & 0x01) {
        int event = TOUCH_NAV;
        xQueueSendFromISR(navQueue, &event, &xHigherPriorityTaskWoken);
    }
    if ((pad_intr >> TOUCH_TLM) & 0x01) {
        int event = TOUCH_TLM;
        xQueueSendFromISR(navQueue, &event, &xHigherPriorityTaskWoken);
    }
    if ((pad_intr >> TOUCH_FS) & 0x01) {
        // --- INÍCIO: INSTRUMENTAÇÃO (Latência) ---
        // Registra o tempo exato em que a interrupção ocorreu.
        fs_trigger_time_us = esp_timer_get_time();
        // --- FIM: INSTRUMENTAÇÃO ---
        vTaskNotifyGiveFromISR(fsHandle, &xHigherPriorityTaskWoken);
    }
    
    // Se uma task de maior prioridade foi acordada (ex: FS_TASK),
    // força uma troca de contexto imediata.
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ---------------------- APP MAIN -------------------------

void app_main(void) {
    // Init touch pads (versão mais robusta)
    ESP_ERROR_CHECK(touch_pad_init());
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    touch_pad_config(TOUCH_NAV, 0);
    touch_pad_config(TOUCH_TLM, 0);
    touch_pad_config(TOUCH_FS, 0);

    touch_pad_filter_start(10);

    // Configura o threshold (limiar) de cada sensor para o disparo da interrupção
    uint16_t touch_value;
    touch_pad_read_filtered(TOUCH_NAV, &touch_value);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_NAV, touch_value * 0.2));
    touch_pad_read_filtered(TOUCH_TLM, &touch_value);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_TLM, touch_value * 0.2));
    touch_pad_read_filtered(TOUCH_FS, &touch_value);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_FS, touch_value * 0.2));

    touch_pad_isr_register(tp_isr, NULL);
    touch_pad_intr_enable();

    // Cria fila para NAV_PLAN
    navQueue = xQueueCreate(10, sizeof(int));

    // Cria as tasks. A ordem de criação não importa, apenas as prioridades.
    // Prioridades: FS(6) > FUS(5) > CTRL(4) > NAV(3) > MONITOR(2)
    xTaskCreate(CTRL_ATT, "ctrl_att", 4096, NULL, 4, &ctrlHandle);
    xTaskCreate(FUS_IMU, "fus_imu", 4096, NULL, 5, &fusHandle);
    xTaskCreate(NAV_PLAN, "nav_plan", 4096, (void*)navQueue, 3, &navHandle);
    xTaskCreate(FS_TASK, "fs_task", 4096, NULL, 6, &fsHandle);
    xTaskCreate(MONITOR_TASK, "monitor_task", 4096, NULL, 2, &monitorHandle);

    printf("Sistema instrumentado iniciado.\n");
}
