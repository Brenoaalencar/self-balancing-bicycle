#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include "driver/ledc.h"

#define SPEED_MODE      LEDC_LOW_SPEED_MODE
#define TIMER_NUM       LEDC_TIMER_0
#define DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define FREQUENCY       50
#define CLK_CONFIG      LEDC_AUTO_CLK
#define CHANNEL         LEDC_CHANNEL_0
#define INTR_TYPE       LEDC_INTR_DISABLE

#define SERVO_MIN_PULSEWIDTH 1120  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 1920  // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE     90    // Maximum angle in degree up to which servo can rotate
#define SERVO_MIN_DEGREE        -90   // Ângulo mínimo
#define SERVO_PIN       GPIO_NUM_5

static const char *TAG_SERVO = "SERVO";

void setup_pwm(uint8_t SERVO_PIN) 
{
    // Configure the LEDC timer
    ledc_timer_config_t ledc_timer = {
    .duty_resolution  = DUTY_RESOLUTION,
    .freq_hz          = FREQUENCY,
    .speed_mode       = SPEED_MODE,
    .timer_num        = TIMER_NUM
    //.clk_cfg          = CLK_CONFIG
    };
    ledc_timer_config(&ledc_timer);

    // Configure the LEDC channel
    ledc_channel_config_t ledc_channel = {
    .channel        = CHANNEL,
    .duty           = 0, // Initial duty cycle
    .gpio_num       = SERVO_PIN,
    .speed_mode     = SPEED_MODE,
    .timer_sel      = TIMER_NUM
    //.hpoint         = 0,
    //.intr_type      = INTR_TYPE
    };
    ledc_channel_config(&ledc_channel);
}

void set_servo_angle(int angle) 
{
    // Convert angle to duty cycle
    int pulsewidth = SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE;
    int duty = (pulsewidth * 8192) / 20000; // 8192 is 2^13 for 13-bit resolution
    ledc_set_duty(SPEED_MODE, CHANNEL, duty);
    ledc_update_duty(SPEED_MODE, CHANNEL);
}

// Task para controlar os servos
void servo_task(void *arg) {
    ESP_LOGI(TAG_SERVO, "Inicializando servos");

    float yaw_angle = 0, pitch_angle = 0;
    float yaw_setpoint = 0, pitch_setpoint = 0;

    while (true) {
        // Obtém os ângulos do filtro de Kalman
        yaw_setpoint = KalmanAngleRoll;
        pitch_setpoint = KalmanAnglePitch;

        // Limita os valores dentro do range permitido
        yaw_angle = fmax(fmin(yaw_setpoint, SERVO_MAX_DEGREE), SERVO_MIN_DEGREE);
        pitch_angle = fmax(fmin(pitch_setpoint, SERVO_MAX_DEGREE), SERVO_MIN_DEGREE);

        ESP_LOGI(TAG_SERVO, "Setpoint Yaw: %.2f, Setpoint Pitch: %.2f", yaw_angle, pitch_angle);

        // Define os valores de comparação para mover os servos
        set_servo_angle(yaw_angle);
        set_servo_angle(pitch_angle);

        // Aguarda 100 ms entre atualizações
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#endif // SERVO_H
