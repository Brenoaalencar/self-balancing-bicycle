#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "motor.hpp"
#include "imu.hpp"
#include "communication.hpp"
#include <array>
#include "driver/gpio.h" 

// Struct para agrupar os objetos usados no controle
struct ControlArgs {
    IMU* imu;
    Motor* left_motor;
    Motor* right_motor;
    Communication* comm;
};

inline float clamp(float value, float min_val, float max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// Função de controle
void control_task(void *param) {
    auto* args = static_cast<ControlArgs*>(param);
    IMU* imu = args->imu;
    Motor* left_motor = args->left_motor;
    Motor* right_motor = args->right_motor;
    Communication* comm = args->comm;

    float e_alpha_i0 = 0.0f, e_y_i0 = 0.0f;
    double alfa_0 = 0, alfa_r = 0.0f, y_r = 0.0f;

    constexpr float R = 0.0325f;
    constexpr float D = 0.18f;
    constexpr float T = 0.01f;

    constexpr float K[2][7] = {
        { 7.014083355278225f,   0.597755525463435f,   0.665560967008112f,   0.009150006651639f, -3.265522897898713f,  -0.012914774343736f,   0.016832704626618 }, // u_vr
        { 7.014083355278217f,   0.597755525463435f,  -0.665560967008094f,  -0.009150006651639f, -3.265522897898709f,   0.012914774343735f,   0.016832704626617 }  // u_vl
    };

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        left_motor->update_velocity();
        right_motor->update_velocity();
        
        float vl = left_motor->get_velocity();
        float vr = right_motor->get_velocity();

        float teta = -imu->roll() * M_PI/180;
        float teta_d = -imu->roll_d() * M_PI/180;

        comm->get_setpoint(alfa_r, y_r);

        float alfa_d = ((vr - vl) / 2.0f) * (R / D);
        float y_d = ((vr + vl) / 2.0f) * R;
        float alfa = alfa_0 + alfa_d * T;
        alfa_0 = alfa;

        float e_alpha = alfa_r - alfa;
        float e_y = y_r - y_d;

        float e_alpha_i = e_alpha + e_alpha_i0;
        float e_y_i = e_y + e_y_i0;

        e_alpha_i0 = e_alpha_i;
        e_y_i0 = e_y_i;

        std::array<float, 7> x = {
            teta, teta_d, alfa, alfa_d, y_d, e_alpha_i, e_y_i
        };

        float u_vr = 0.0f;
        float u_vl = 0.0f;
        for (int i = 0; i < 7; ++i) {
            u_vr -= K[0][i] * x[i];
            u_vl -= K[1][i] * x[i];
        }

        u_vr = clamp(u_vr, -1.0f, 1.0f);
        u_vl = clamp(u_vl, -1.0f, 1.0f);

        // Log dos valores de controle e vetor de estado
        //ESP_LOGI("CONTROL", "u_vl: %.4f, u_vr: %.4f, x: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",u_vl, u_vr, x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

        left_motor->set_duty(u_vl);
        right_motor->set_duty(u_vr);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main() {
    // Configura GPIO2 (D2) e GPIO26 como saída e nível alto
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);

    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_27, 1);

    // IMU
    static IMU imu;
    imu.init();
    imu.set_verbose(false);

    // Comunicação
    static Communication comm;
    xTaskCreate(&Communication::task, "communication_task", 8192, &comm, 5, NULL);

    auto monitor_task = [](void *param) {
        Communication *c = static_cast<Communication*>(param);
        while (true) {
            double a, y;
            c->get_setpoint(a, y);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    };
    //xTaskCreate(monitor_task, "monitor_task", 4096, &comm, 4, NULL);

    // Motores
    static Motor right_motor(GPIO_NUM_4, GPIO_NUM_15, 2024, "RIGHT",
                            GPIO_NUM_33, LEDC_CHANNEL_0, LEDC_TIMER_0,
                            GPIO_NUM_26, GPIO_NUM_25); //2112 = 11 * 4 * 46

    static Motor left_motor(GPIO_NUM_16, GPIO_NUM_17, 2024, "LEFT",
                             GPIO_NUM_13, LEDC_CHANNEL_1, LEDC_TIMER_0,
                             GPIO_NUM_14, GPIO_NUM_12);

    right_motor.begin();
    left_motor.begin();

    right_motor.set_verbose(false);
    left_motor.set_verbose(false);

    static ControlArgs control_args = {
        &imu, &left_motor, &right_motor, &comm
    };

    xTaskCreate(control_task, "control_task", 8192, &control_args, 6, NULL);
    // Teste PWM
    // right_motor.set_duty(0.7461); //para 6.28 rad/s com 5V da USB (1 rotação/s)
    // while(1){
    //    right_motor.update_velocity();
    //    vTaskDelay(pdMS_TO_TICKS(10));
    // }
    
}