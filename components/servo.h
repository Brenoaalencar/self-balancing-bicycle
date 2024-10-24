#ifndef SERVO_H
#define SERVO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/mcpwm_prelude.h"
#include "imu.h"  // Para acessar os ângulos do Kalman
#include <math.h>

#define SERVO_MIN_PULSEWIDTH_US 500  // Pulso mínimo em microsegundos
#define SERVO_MAX_PULSEWIDTH_US 2500  // Pulso máximo em microsegundos
#define SERVO_MIN_DEGREE        -90   // Ângulo mínimo
#define SERVO_MAX_DEGREE        90    // Ângulo máximo

#define SERVO_YAW_GPIO          2     // GPIO do servo Yaw
#define SERVO_PITCH_GPIO        4     // GPIO do servo Pitch
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1 MHz, 1 us por tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20 ms

static const char *TAG_SERVO = "SERVO";

// Converte ângulo para valor de comparação do MCPWM
static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Inicializa MCPWM para um servo
void init_servo(mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, 
                mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator, int gpio_num) {

    // Configuração do timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .intr_priority = 0,  // Adicionando intr_priority faltante
        .flags = {0}         // Adicionando flags faltantes
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, timer));

    // Configuração do operador
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
        .intr_priority = 0,  // Prioridade da interrupção
        .flags = {0},        // Definição de flags
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, *timer));

    // Configuração do comparador
    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true,  // Inicializando corretamente o campo
        }
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(*oper, &comparator_config, comparator));

    // Configuração do gerador
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
        .flags = {0},  // Definição de flags
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(*oper, &generator_config, generator));

    // Define o valor inicial de comparação (ângulo 0)
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*comparator, angle_to_compare(0)));

    // Ações do gerador
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(*generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(*generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW)));

    // Ativa e inicia o timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(*timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP));
}

// Task para controlar os servos
void servo_task(void *arg) {
    ESP_LOGI(TAG_SERVO, "Inicializando servos");

    mcpwm_timer_handle_t yaw_timer, pitch_timer;
    mcpwm_oper_handle_t yaw_oper, pitch_oper;
    mcpwm_cmpr_handle_t yaw_comparator, pitch_comparator;
    mcpwm_gen_handle_t yaw_generator, pitch_generator;

    // Inicializa os servos Yaw e Pitch
    init_servo(&yaw_timer, &yaw_oper, &yaw_comparator, &yaw_generator, SERVO_YAW_GPIO);
    init_servo(&pitch_timer, &pitch_oper, &pitch_comparator, &pitch_generator, SERVO_PITCH_GPIO);

    float yaw_angle = 0, pitch_angle = 0;
    float yaw_setpoint = 0, pitch_setpoint = 0;
    
    while (true) {
        // Obtém os ângulos do filtro de Kalman
        yaw_setpoint = KalmanAngleRoll;
        pitch_setpoint = KalmanAnglePitch;

        // Limita os valores dentro do range permitido
        yaw_angle = fmax(fmin(yaw_setpoint, SERVO_MAX_DEGREE), SERVO_MIN_DEGREE);
        pitch_angle = fmax(fmin(pitch_setpoint, SERVO_MAX_DEGREE), SERVO_MIN_DEGREE);

        ESP_LOGI(TAG_SERVO, "Setpoint Yaw: %f, Setpoint Pitch: %f", yaw_angle, pitch_angle);

        // Define os valores de comparação para mover os servos
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(yaw_comparator, angle_to_compare((int)yaw_angle)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pitch_comparator, angle_to_compare((int)pitch_angle)));

        // Aguarda 100 ms entre atualizações
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#endif  // SERVO_H
