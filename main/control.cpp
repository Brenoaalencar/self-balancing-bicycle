#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define CONTROL_PERIOD_MS 10  // Período do controlador

//SemaphoreHandle_t sensor_data_ready;  // Semáforo para sincronização dos sensores
//SemaphoreHandle_t setpoint_mutex;  // Mutex para proteger o setpoint

float sensor_data = 0;  // Simulação de leitura do sensor
float setpoint = 0;  // Setpoint atualizado pela comunicação

// Simulação da leitura dos sensores
void read_sensors() {
    sensor_data += 0.1;  // Simula uma leitura variando
    //pega as infos dos sensores e adiciona nas variáveis
}

// Task de aquisição dos sensores
void sensor_task(void *pvParameters) {
    while (1) {
        read_sensors();  // Lê sensores
        //xSemaphoreGive(sensor_data_ready);  // Sinaliza que os dados estão prontos
        vTaskDelay(pdMS_TO_TICKS(1));  // Pequeno delay para evitar sobrecarga
    }
}


// Task de controle com execução periódica
void control_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Espera os sensores estarem prontos
        //if (xSemaphoreTake(sensor_data_ready, pdMS_TO_TICKS(CONTROL_PERIOD_MS/2))) {
            float sp;

            /* Protege o acesso ao setpoint com mutex
            if (xSemaphoreTake(setpoint_mutex, pdMS_TO_TICKS(5))) {
                sp = setpoint;
                xSemaphoreGive(setpoint_mutex);
            } else {
                sp = 0;  // Se não conseguir acessar, mantém um valor seguro
            }
            */
            float error = sp - sensor_data;
            float u = -0.5 * error;  // Simulação de controle PID simples
            ESP_LOGI("CONTROL", "Setpoint: %.2f | Sensor: %.2f | Controle: %.2f", sp, sensor_data, u);
        //}

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Task de comunicação (serial/Bluetooth)
void communication_task(void *pvParameters) {
    while (1) {
        // Simulação: Recebe um setpoint aleatório a cada 2-5 segundos
        vTaskDelay(pdMS_TO_TICKS((rand() % 3000) + 2000));

        float new_setpoint = (rand() % 100) / 10.0;  // Novo setpoint aleatório

        /* Protege o acesso ao setpoint com mutex
        if (xSemaphoreTake(setpoint_mutex, pdMS_TO_TICKS(5))) {
            setpoint = new_setpoint;
            xSemaphoreGive(setpoint_mutex);
        }
        */

        ESP_LOGI("COMM", "Novo Setpoint recebido: %.2f", new_setpoint);
    }
}

void app_main() {
    //sensor_data_ready = xSemaphoreCreateBinary();
    //setpoint_mutex = xSemaphoreCreateMutex();

    // Criar tasks
    xTaskCreatePinnedToCore(sensor_task, "Sensor Task", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "Control Task", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(communication_task, "Comm Task", 2048, NULL, 1, NULL, 0);
}
