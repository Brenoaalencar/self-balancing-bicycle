#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


//static const char *TAG = "self-balanced";

extern "C" void app_main() {
    // Inicializa a task dos servos
    //xTaskCreate(servo_task, "servo_task", 4096, NULL, 5, NULL);
    // Criação da task de leitura da IMU
    //xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}