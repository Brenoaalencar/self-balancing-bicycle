#ifndef IMU_H
#define IMU_H

#include "i2c_bus.h"
#include "esp_timer.h"
#include <esp_log.h>
#include <math.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MPU9250_ADDR 0x68  // Endereço do MPU9250

// Variáveis globais para o Kalman e as leituras
static float RateRoll, RatePitch, RateYaw;
static float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
static float AccX, AccY, AccZ;
static float AngleRoll, AnglePitch;
static uint32_t LoopTimer;
static float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
static float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
static float Kalman1DOutput[] = {0, 0};

static const char *IMU_TAG = "IMU";


void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    // Leitura dos dados do acelerômetro (registrador 0x3B)
    mpu9250_register_read(0x3B, accel_data, 6);

    // Converter dados do acelerômetro
    int16_t AccXLSB = (accel_data[0] << 8) | accel_data[1];
    int16_t AccYLSB = (accel_data[2] << 8) | accel_data[3];
    int16_t AccZLSB = (accel_data[4] << 8) | accel_data[5];

    // Leitura dos dados do giroscópio (registrador 0x43)
    mpu9250_register_read(0x43, gyro_data, 6);

    // Converter dados do giroscópio
    int16_t GyroX = (gyro_data[0] << 8) | gyro_data[1];
    int16_t GyroY = (gyro_data[2] << 8) | gyro_data[3];
    int16_t GyroZ = (gyro_data[4] << 8) | gyro_data[5];

    // Cálculo das taxas
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    // Cálculo das acelerações
    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    // Cálculo dos ângulos
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / M_PI;
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / M_PI;
}

void imu_task(void *pvParameters) {
    // Inicializa o I2C e o MPU9250
    ESP_ERROR_CHECK(i2c_master_init());

    // Acordar o MPU9250 (registrador 0x6B)
    ESP_ERROR_CHECK(mpu9250_register_write_byte(0x6B, 0x00));

    // Calibração inicial
    for (int i = 0; i < 2000; i++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;

    LoopTimer = esp_timer_get_time();  // Usa o timer do ESP32

    while (1) {
        gyro_signals();

        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;

        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

        ESP_LOGI(IMU_TAG, "Roll Angle [°]: %.2f, Pitch Angle [°]: %.2f", KalmanAngleRoll, KalmanAnglePitch);

        // Controle de tempo de loop para 4ms
        while ((esp_timer_get_time() - LoopTimer) < 4000);
        LoopTimer = esp_timer_get_time();
    }
}

#endif // IMU_H
