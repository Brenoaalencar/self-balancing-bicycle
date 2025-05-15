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
    void set_duty(float u); // u em faixa -4096 a 4096
    void update_velocity();
    float get_velocity() const;
    void set_verbose(bool v);

private:
    gpio_num_t _pinA, _pinB;
    int _pulsos_por_rotacao;
    std::string _nome;

    gpio_num_t _pwmPin;
    ledc_channel_t _pwmChannel;
    ledc_timer_t _pwmTimer;

    gpio_num_t _in1Pin, _in2Pin;

    pcnt_unit_t _pcnt_unit;
    int16_t _prev_pulses = 0;
    int16_t _curr_pulses = 0;
    float _velocity = 0.0f; // rad/s
    float _prev_velocity = 0.5f; // rad/s
    float _last_duty=0.0f;
    bool _verbose = false;
};

#endif
