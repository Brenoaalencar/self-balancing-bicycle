#include "imu.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>

static const char* TAG = "IMU";
constexpr float DEADZONE_THRESHOLD = 0.05f; // rad/s
constexpr float DT = 0.01f; // 10 ms
constexpr float ACCEL_UNCERTAINTY = 3.0f;
constexpr float GYRO_UNCERTAINTY = 4.0f;

IMU::IMU()
    : rate_roll(0), rate_pitch(0), rate_yaw(0),
      acc_x(0), acc_y(0), acc_z(0),
      angle_roll(0), angle_pitch(0),
      kalman_roll(0), kalman_pitch(0),
      kalman_unc_roll(4.0f), kalman_unc_pitch(4.0f),
      last_roll(0), last_pitch(0),
      roll_rate(0), pitch_rate(0),
      filt_roll_rate(0), filt_pitch_rate(0),
      kalman_rate_roll(0), kalman_rate_pitch(0),
      kalman_unc_rate_roll(4.0f), kalman_unc_rate_pitch(4.0f),
      verbose(false) {}

void IMU::set_verbose(bool v) {
    verbose = v;
}

void IMU::init() {
    esp_err_t ret;

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

    ret = mpu9250_register_write_byte(0x6B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 not found or failed to wake up (0x%x)", ret);
        return;
    }

    // Configura acelerômetro para ±2g (ACCEL_CONFIG = 0x1C, valor = 0x00)
    ret = mpu9250_register_write_byte(0x1C, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer range (0x%x)", ret);
        return;
    }

    // Configura giroscópio para ±500°/s (GYRO_CONFIG = 0x1B, valor = 0x08)
    ret = mpu9250_register_write_byte(0x1B, 0x08);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope range (0x%x)", ret);
        return;
    }

    nvs_handle_t nvs_handle;
    ret = nvs_open("calibration", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace (0x%x)", ret);
        return;
    }

    uint8_t calibrated = 0;
    ret = nvs_get_u8(nvs_handle, "calibrated", &calibrated);

    if (ret == ESP_OK && calibrated == 1) {
        size_t size = sizeof(float);
        if (nvs_get_blob(nvs_handle, "rate_roll", &rate_roll, &size) != ESP_OK ||
            nvs_get_blob(nvs_handle, "rate_pitch", &rate_pitch, &size) != ESP_OK ||
            nvs_get_blob(nvs_handle, "rate_yaw", &rate_yaw, &size) != ESP_OK ||
            nvs_get_blob(nvs_handle, "acc_x", &acc_offset_x, &size) != ESP_OK ||
            nvs_get_blob(nvs_handle, "acc_y", &acc_offset_y, &size) != ESP_OK ||
            nvs_get_blob(nvs_handle, "acc_z", &acc_offset_z, &size) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load calibration data. Resetting calibration flag.");
            calibrated = 0;
            nvs_set_u8(nvs_handle, "calibrated", 0);
            nvs_commit(nvs_handle);
        } else {
            ESP_LOGI(TAG, "Loaded calibration from flash: rate_roll=%.2f rate_pitch=%.2f rate_yaw=%.2f acc_x=%.2f acc_y=%.2f acc_z=%.2f", rate_roll, rate_pitch, rate_yaw, acc_offset_x, acc_offset_y, acc_offset_z);
        }
    }

    if (calibrated != 1) {
        ESP_LOGI(TAG, "No calibration found. Starting calibration...");

        float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
        float sum_ax = 0, sum_ay = 0, sum_az = 0;

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

            int16_t ax = (a[0] << 8) | a[1];
            int16_t ay = (a[2] << 8) | a[3];
            int16_t az = (a[4] << 8) | a[5];

            sum_ax += ax / ACCEL_SCALE;
            sum_ay += ay / ACCEL_SCALE;
            sum_az += az / ACCEL_SCALE;

            sum_roll += gx / GYRO_SCALE;
            sum_pitch += gy / GYRO_SCALE;
            sum_yaw += gz / GYRO_SCALE;

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        rate_roll = sum_roll / 2000;
        rate_pitch = sum_pitch / 2000;
        rate_yaw = sum_yaw / 2000;

        acc_offset_x = sum_ax / 2000;
        acc_offset_y = sum_ay / 2000;
        acc_offset_z = (sum_az / 2000) - 1.0f; // Ajuste para gravidade

        nvs_set_blob(nvs_handle, "rate_roll", &rate_roll, sizeof(float));
        nvs_set_blob(nvs_handle, "rate_pitch", &rate_pitch, sizeof(float));
        nvs_set_blob(nvs_handle, "rate_yaw", &rate_yaw, sizeof(float));
        nvs_set_blob(nvs_handle, "acc_x", &acc_offset_x, sizeof(float));
        nvs_set_blob(nvs_handle, "acc_y", &acc_offset_y, sizeof(float));
        nvs_set_blob(nvs_handle, "acc_z", &acc_offset_z, sizeof(float));
        nvs_set_u8(nvs_handle, "calibrated", 1);
        nvs_commit(nvs_handle);

        ESP_LOGI(TAG, "Calibration complete and saved: roll=%.2f pitch=%.2f yaw=%.2f", rate_roll, rate_pitch, rate_yaw);
    }

    nvs_close(nvs_handle);

    xTaskCreatePinnedToCore(task_imu, "imu_task", 4096, this, 1, nullptr, 1);
}

float IMU::roll() const { return kalman_roll; }
float IMU::pitch() const { return kalman_pitch; }
float IMU::roll_d() const { return kalman_rate_roll; }
float IMU::pitch_d() const { return kalman_rate_pitch; }

void IMU::kalman_1d(float& state, float& uncertainty, float input, float measurement,
                    float& output, float& out_uncertainty) {
    state += DT * input;
    uncertainty += pow(DT * GYRO_UNCERTAINTY, 2);
    float gain = uncertainty / (uncertainty + pow(ACCEL_UNCERTAINTY, 2));
    state += gain * (measurement - state);
    uncertainty *= (1 - gain);
    output = state;
    out_uncertainty = uncertainty;
}

void IMU::task_imu(void* pvParams) {
    IMU* imu = static_cast<IMU*>(pvParams);
    uint64_t last_time = esp_timer_get_time();
    const float beta = 0.5f;

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

        imu->acc_x = ax / ACCEL_SCALE - imu->acc_offset_x;;
        imu->acc_y = ay / ACCEL_SCALE - imu->acc_offset_y;
        imu->acc_z = az / ACCEL_SCALE - imu->acc_offset_z;

        imu->rate_roll = (gx / GYRO_SCALE) - imu->rate_roll;
        imu->rate_pitch = (gy / GYRO_SCALE) - imu->rate_pitch;
        imu->rate_yaw = (gz / GYRO_SCALE) - imu->rate_yaw;

        imu->angle_roll = atan(imu->acc_y / sqrtf(imu->acc_x * imu->acc_x + imu->acc_z * imu->acc_z)) * 180.0f / M_PI;
        imu->angle_pitch = -atan(imu->acc_x / sqrtf(imu->acc_y * imu->acc_y + imu->acc_z * imu->acc_z)) * 180.0f / M_PI;

        imu->kalman_1d(imu->kalman_roll, imu->kalman_unc_roll, imu->rate_roll, imu->angle_roll, imu->kalman_roll, imu->kalman_unc_roll);
        imu->kalman_1d(imu->kalman_pitch, imu->kalman_unc_pitch, imu->rate_pitch, imu->angle_pitch, imu->kalman_pitch, imu->kalman_unc_pitch);

        imu->kalman_1d(imu->kalman_rate_roll, imu->kalman_unc_rate_roll,
                       0.0f, imu->rate_roll, imu->kalman_rate_roll, imu->kalman_unc_rate_roll);

        imu->kalman_1d(imu->kalman_rate_pitch, imu->kalman_unc_rate_pitch,
                       0.0f, imu->rate_pitch, imu->kalman_rate_pitch, imu->kalman_unc_rate_pitch);

        uint64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1e6f;
        imu->roll_rate = (imu->kalman_roll - imu->last_roll) / dt;
        imu->pitch_rate = (imu->kalman_pitch - imu->last_pitch) / dt;
        imu->last_roll = imu->kalman_roll;
        imu->last_pitch = imu->kalman_pitch;
        last_time = now;

        imu->filt_roll_rate  = beta * imu->rate_roll  + (1 - beta) * imu->roll_rate;
        imu->filt_pitch_rate = beta * imu->rate_pitch + (1 - beta) * imu->pitch_rate;

        if (fabsf(imu->kalman_rate_roll) < DEADZONE_THRESHOLD) {
            imu->filt_roll_rate = 0.0f;
        }
        if (fabsf(imu->filt_pitch_rate) < DEADZONE_THRESHOLD) {
            imu->filt_pitch_rate = 0.0f;
        }

        if (imu->verbose) {
            ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f | Roll_d: %.2f, Pitch_d: %.2f",
                     imu->kalman_roll, imu->kalman_pitch, imu->kalman_rate_roll, imu->kalman_rate_pitch);
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}
