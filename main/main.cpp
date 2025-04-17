#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "motor.hpp"
#include "imu.hpp"
#include "communication.hpp"
#include <tuple>
#include <array>

// Função de controle
void control_task(void *param) {
    auto args = static_cast<std::tuple<IMU*, Motor*, Motor*, Communication*>*>(param);
    IMU* imu = std::get<0>(*args);
    Motor* left_motor = std::get<1>(*args);
    Motor* right_motor = std::get<2>(*args);
    Communication* comm = std::get<3>(*args);

    float e_alpha_int = 0.0f, e_y_int = 0.0f;
    double alfa_r = 0.0f, y_r = 0.0f;

    constexpr float K[7] = { 30.0f, 1.0f, 10.0f, 0.2f, 2.0f, 0.1f, 0.5f };

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        left_motor->update_velocity();
        right_motor->update_velocity();

        float teta = imu->pitch();       // θ
        float teta_d = imu->pitch_d();   // θ̇

        comm->get_setpoint(alfa_r, y_r);
        float alfa = alfa_r;
        float y = y_r;

        float alfa_d = (right_motor->get_velocity() - left_motor->get_velocity()) / 2.0f;
        float y_d = (right_motor->get_velocity() + left_motor->get_velocity()) / 2.0f;

        float e_alpha = alfa_r - alfa;
        float e_y = y_r - y;

        e_alpha_int += e_alpha * 0.01f;
        e_y_int += e_y * 0.01f;

        std::array<float, 7> x = {
            teta,
            teta_d,
            alfa,
            alfa_d,
            y,
            y_d,
            e_alpha_int + e_y_int
        };

        float u = 0.0f;
        for (int i = 0; i < 7; ++i) {
            u -= K[i] * x[i];
        }

        float u_vr = std::min(std::max(u, -4095.0f), 4095.0f);
        float u_vl = u_vr;  // Pode adaptar para split se desejar

        left_motor->set_duty(u_vl*1000);
        right_motor->set_duty(u_vr);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}

extern "C" void app_main() {
    // IMU
    static IMU imu;
    //imu.init();

    // Comunicação
    static Communication comm;
    xTaskCreate(&Communication::task, "communication_task", 8192, &comm, 5, NULL);

    auto monitor_task = [](void *param) {
        Communication *c = static_cast<Communication*>(param);
        while (true) {
            double a, y;
            c->get_setpoint(a, y);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    };
    xTaskCreate(monitor_task, "monitor_task", 4096, &comm, 4, NULL);

    // Motores
    static Motor left_motor(GPIO_NUM_17, GPIO_NUM_16, 10, "LEFT",
                            GPIO_NUM_33, LEDC_CHANNEL_0, LEDC_TIMER_0,
                            GPIO_NUM_25, GPIO_NUM_26);

    static Motor right_motor(GPIO_NUM_34, GPIO_NUM_35, 10, "RIGHT",
                             GPIO_NUM_32, LEDC_CHANNEL_1, LEDC_TIMER_0,
                             GPIO_NUM_27, GPIO_NUM_14);

    left_motor.begin();
    right_motor.begin();

    left_motor.set_verbose(true);
    right_motor.set_verbose(false);

    left_motor.set_verbose(true);
    left_motor.set_duty(3000);  // Gira pra frente
vTaskDelay(pdMS_TO_TICKS(3000));
left_motor.set_duty(-3000); // Gira pra trás
vTaskDelay(pdMS_TO_TICKS(3000));
left_motor.set_duty(0);     // Parar


    // Tupla para passar para a task
    static std::tuple<IMU*, Motor*, Motor*, Communication*> control_args = {
        &imu, &left_motor, &right_motor, &comm
    };
    xTaskCreate(control_task, "control_task", 8192, &control_args, 6, NULL);
}
