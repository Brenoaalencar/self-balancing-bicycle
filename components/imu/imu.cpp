#include "imu.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"


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
    esp_err_t ret;

    // Inicializa a NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: 0x%x", ret);
        return;
    }

    ret = mpu9250_register_write_byte(0x6B, 0x00); // Wake up MPU9250
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 not found or failed to wake up (0x%x)", ret);
        return;
    }

    // Abre namespace "calibration"
    nvs_handle_t nvs_handle;
    ret = nvs_open("calibration", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace (0x%x)", ret);
        return;
    }

    // Verifica se já existe uma calibração feita
    uint8_t calibrated = 0;
    ret = nvs_get_u8(nvs_handle, "calibrated", &calibrated);

    if (ret == ESP_OK && calibrated == 1) {
        // Já calibrado, então apenas carrega os dados
        nvs_get_i32(nvs_handle, "rate_roll", reinterpret_cast<int32_t*>(&rate_roll));
        nvs_get_i32(nvs_handle, "rate_pitch", reinterpret_cast<int32_t*>(&rate_pitch));
        nvs_get_i32(nvs_handle, "rate_yaw", reinterpret_cast<int32_t*>(&rate_yaw));

        ESP_LOGI(TAG, "Loaded calibration from flash: roll=%.2f pitch=%.2f yaw=%.2f", rate_roll, rate_pitch, rate_yaw);

    } else {
        // Se não calibrado, roda a calibração agora
        ESP_LOGI(TAG, "No calibration found. Starting calibration...");

        float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;

        for (int i = 0; i < 2000; i++) {
            uint8_t a[6], g[6];
            if (mpu9250_register_read(0x3B, a, 6) != ESP_OK ||
                mpu9250_register_read(0x43, g, 6) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read IMU data during calibration at iteration %d", i);
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            }

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

        // Salva na NVS
        nvs_set_i32(nvs_handle, "rate_roll", static_cast<int32_t>(rate_roll));
        nvs_set_i32(nvs_handle, "rate_pitch", static_cast<int32_t>(rate_pitch));
        nvs_set_i32(nvs_handle, "rate_yaw", static_cast<int32_t>(rate_yaw));
        nvs_set_u8(nvs_handle, "calibrated", 1);
        nvs_commit(nvs_handle);

        ESP_LOGI(TAG, "Calibration complete and saved: roll=%.2f pitch=%.2f yaw=%.2f", rate_roll, rate_pitch, rate_yaw);
    }

    nvs_close(nvs_handle);

    // Task de leitura contínua da IMU
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
