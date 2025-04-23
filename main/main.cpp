#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "motor.hpp"
#include "imu.hpp"
#include "communication.hpp"
#include <array>

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

    // Estados integradores
    float e_alpha_i0 = 0.0f, e_y_i0 = 0.0f;

    // Setpoints
    double alfa_r = 0.0f, y_r = 0.0f;

    constexpr float R = 0.0325f; //0.065/2
    constexpr float D = 0.19f;
    constexpr float T = 0.01f; // 10ms

    // K[linha][estado]
    constexpr float K[2][7] = {
        { -36.4921f, -3.2732f, 6.7180f, 0.0059f, -22.2794f, -0.1827f, 0.1390f }, // u_vr
        { -36.4921f, -3.2732f, -6.7180f, -0.0059f, -22.2794f, 0.1827f, 0.1390f }  // u_vl
    };

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        left_motor->update_velocity();
        right_motor->update_velocity();

        // Leitura dos sensores
        float teta = imu->pitch();      // θ
        float teta_d = imu->pitch_d();  // θ̇

        // Setpoints recebidos
        comm->get_setpoint(alfa_r, y_r);

        // Velocidades dos motores
        float vl = left_motor->get_velocity();
        float vr = right_motor->get_velocity();

        // Estados derivados das velocidades
        float alfa_d = (vr - vl) / 2.0f * (R / D);  // estado de guinada (rad/s)
        float y_d = (vr + vl) / 2.0f * R;           // estado de avanço (m/s)

        float alfa = alfa_d * T; // integração simples (alfa = ∫alfȧ dt)

        // Erros
        float e_alpha = 8191*alfa_r - alfa;
        float e_y = 8191*y_r - y_d;

        // Integrais dos erros
        float e_alpha_i = e_alpha + e_alpha_i0;
        float e_y_i = e_y + e_y_i0;

        // Atualiza os acumuladores
        e_alpha_i0 = e_alpha_i;
        e_y_i0 = e_y_i;

        // Vetor de estados
        std::array<float, 7> x = {
            teta,
            teta_d,
            alfa,
            alfa_d,
            y_d,
            e_alpha_i,
            e_y_i
        };

        // Controle para u_vr e u_vl
        float u_vr = 0.0f;
        float u_vl = 0.0f;
        for (int i = 0; i < 7; ++i) {
            u_vr -= K[0][i] * x[i];
            u_vl -= K[1][i] * x[i];
        }

        // Saturação
        u_vr = clamp(u_vr, -8191.0f, 8191.0f);
        u_vl = clamp(u_vl, -8191.0f, 8191.0f);
        
        // Seta PWM
        left_motor->set_duty(u_vl);
        right_motor->set_duty(u_vr);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    // IMU
    static IMU imu;
    imu.init();
    imu.set_verbose(true);

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

    left_motor.set_verbose(false);
    right_motor.set_verbose(false);

    //left_motor.set_verbose(true);
    //left_motor.set_duty(3000);  // Gira pra frente
    //vTaskDelay(pdMS_TO_TICKS(3000));
    //left_motor.set_duty(-3000); // Gira pra trás
    //vTaskDelay(pdMS_TO_TICKS(3000));
    //left_motor.set_duty(0);     // Parar

    // Struct com ponteiros para passar à tarefa
    static ControlArgs control_args = {
        &imu, &left_motor, &right_motor, &comm
    };

    xTaskCreate(control_task, "control_task", 8192, &control_args, 6, NULL);

    // Teste PWM
    //left_motor.set_duty(4539);
    //while(1){
    //    left_motor.update_velocity();
    //    vTaskDelay(pdMS_TO_TICKS(100));
    //}
}
