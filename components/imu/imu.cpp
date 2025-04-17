#include "imu.hpp"
#include "esp_log.h"

static const char* TAG = "IMU";

IMU::IMU()
    : rate_roll(0), rate_pitch(0), rate_yaw(0),
      acc_x(0), acc_y(0), acc_z(0),
      angle_roll(0), angle_pitch(0),
      kalman_roll(0), kalman_pitch(0),
      kalman_unc_roll(4.0f), kalman_unc_pitch(4.0f),
      last_roll(0), last_pitch(0),
      roll_rate(0), pitch_rate(0),
      verbose(false) {}

void IMU::set_verbose(bool v) {
    verbose = v;
}

void IMU::init() {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu9250_register_write_byte(0x6B, 0x00)); // Wake MPU9250

    float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;

    for (int i = 0; i < 2000; i++) {
        uint8_t a[6], g[6];
        mpu9250_register_read(0x3B, a, 6);
        mpu9250_register_read(0x43, g, 6);

        int16_t gx = (g[0] << 8) | g[1];
        int16_t gy = (g[2] << 8) | g[3];
        int16_t gz = (g[4] << 8) | g[5];

        sum_roll += gx / GYRO_SCALE;
        sum_pitch += gy / GYRO_SCALE;
        sum_yaw += gz / GYRO_SCALE;

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    rate_roll = sum_roll / 2000;
    rate_pitch = sum_pitch / 2000;
    rate_yaw = sum_yaw / 2000;

    xTaskCreatePinnedToCore(task_imu, "imu_task", 4096, this, 1, nullptr, 1);
}

float IMU::roll() const { return kalman_roll; }
float IMU::pitch() const { return kalman_pitch; }
float IMU::roll_d() const { return roll_rate; }
float IMU::pitch_d() const { return pitch_rate; }

void IMU::kalman_1d(float& state, float& uncertainty, float input, float measurement, float& output, float& out_uncertainty) {
    state += 0.004f * input;
    uncertainty += 0.004f * 0.004f * 16.0f;
    float gain = uncertainty / (uncertainty + 9.0f);
    state += gain * (measurement - state);
    uncertainty *= (1 - gain);
    output = state;
    out_uncertainty = uncertainty;
}

void IMU::task_imu(void* pvParams) {
    IMU* imu = static_cast<IMU*>(pvParams);
    uint64_t last_time = esp_timer_get_time();

    while (true) {
        uint8_t accel[6], gyro[6];
        mpu9250_register_read(0x3B, accel, 6);
        mpu9250_register_read(0x43, gyro, 6);

        int16_t ax = (accel[0] << 8) | accel[1];
        int16_t ay = (accel[2] << 8) | accel[3];
        int16_t az = (accel[4] << 8) | accel[5];

        int16_t gx = (gyro[0] << 8) | gyro[1];
        int16_t gy = (gyro[2] << 8) | gyro[3];
        int16_t gz = (gyro[4] << 8) | gyro[5];

        imu->acc_x = ax / ACCEL_SCALE;
        imu->acc_y = ay / ACCEL_SCALE;
        imu->acc_z = az / ACCEL_SCALE;

        imu->rate_roll = (gx / GYRO_SCALE) - imu->rate_roll;
        imu->rate_pitch = (gy / GYRO_SCALE) - imu->rate_pitch;
        imu->rate_yaw = (gz / GYRO_SCALE) - imu->rate_yaw;

        imu->angle_roll = atan(imu->acc_y / sqrtf(imu->acc_x * imu->acc_x + imu->acc_z * imu->acc_z)) * 180.0f / M_PI;
        imu->angle_pitch = -atan(imu->acc_x / sqrtf(imu->acc_y * imu->acc_y + imu->acc_z * imu->acc_z)) * 180.0f / M_PI;

        imu->kalman_1d(imu->kalman_roll, imu->kalman_unc_roll, imu->rate_roll, imu->angle_roll, imu->kalman_roll, imu->kalman_unc_roll);
        imu->kalman_1d(imu->kalman_pitch, imu->kalman_unc_pitch, imu->rate_pitch, imu->angle_pitch, imu->kalman_pitch, imu->kalman_unc_pitch);

        // Derivadas
        uint64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1e6f; // segundos
        imu->roll_rate = (imu->kalman_roll - imu->last_roll) / dt;
        imu->pitch_rate = (imu->kalman_pitch - imu->last_pitch) / dt;
        imu->last_roll = imu->kalman_roll;
        imu->last_pitch = imu->kalman_pitch;
        last_time = now;

        if (imu->verbose) {
            ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f | Roll_d: %.2f, Pitch_d: %.2f",
                     imu->kalman_roll, imu->kalman_pitch, imu->roll_rate, imu->pitch_rate);
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}
