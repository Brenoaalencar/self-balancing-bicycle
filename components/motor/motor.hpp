#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>

class Motor {
public:
    Motor(gpio_num_t pinA, gpio_num_t pinB, int pulsos_por_rotacao, const std::string& nome,
          gpio_num_t pwmPin, ledc_channel_t pwmChannel, ledc_timer_t pwmTimer,
          gpio_num_t in1Pin, gpio_num_t in2Pin);

    void begin();
    void set_duty(float u);             // u em faixa -1.0 a 1.0
    void update_velocity();             // Atualiza velocidade angular (rad/s)
    float get_velocity() const;         // Retorna velocidade filtrada
    void set_verbose(bool v);           // Habilita logs

private:
    // Entradas do encoder
    gpio_num_t _pinA, _pinB;
    int _pulsos_por_rotacao;
    std::string _nome;

    // PWM e direção
    gpio_num_t _pwmPin;
    ledc_channel_t _pwmChannel;
    ledc_timer_t _pwmTimer;
    gpio_num_t _in1Pin, _in2Pin;

    // Contador de pulsos
    pcnt_unit_t _pcnt_unit;
    int16_t _prev_pulses = 0;
    int32_t _total_pulses = 0;

    // Velocidade
    float _velocity = 0.0f;     // filtrada
    float _velocity_raw = 0.0f; // bruta (sem filtro)

    // Tempo
    int64_t _last_time = 0;     // microssegundos

    // Log
    bool _verbose = false;
};

#endif
