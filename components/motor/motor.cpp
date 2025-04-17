#include "motor.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "Motor";

Motor::Motor(gpio_num_t pinA, gpio_num_t pinB, int pulsos_por_rotacao, const std::string& nome,
             gpio_num_t pwmPin, ledc_channel_t pwmChannel, ledc_timer_t pwmTimer,
             gpio_num_t in1Pin, gpio_num_t in2Pin)
    : _pinA(pinA), _pinB(pinB), _pulsos_por_rotacao(pulsos_por_rotacao), _nome(nome),
      _pwmPin(pwmPin), _pwmChannel(pwmChannel), _pwmTimer(pwmTimer),
      _in1Pin(in1Pin), _in2Pin(in2Pin)
{}

void Motor::begin() {
    // Configura pinos de direção
    gpio_config_t dir_conf = {};
    dir_conf.mode = GPIO_MODE_OUTPUT;
    dir_conf.pin_bit_mask = (1ULL << _in1Pin) | (1ULL << _in2Pin);
    gpio_config(&dir_conf);
    gpio_set_level(_in1Pin, 0);
    gpio_set_level(_in2Pin, 0);

    // PWM - ordem correta dos campos
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = _pwmTimer,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = _pwmPin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = _pwmChannel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = _pwmTimer,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    

    // PCNT
    _pcnt_unit = (_pwmChannel == LEDC_CHANNEL_0) ? PCNT_UNIT_0 : PCNT_UNIT_1;

    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = _pinA;
    pcnt_config.ctrl_gpio_num = _pinB;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = _pcnt_unit;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.counter_h_lim = 10000;
    pcnt_config.counter_l_lim = -10000;
    pcnt_unit_config(&pcnt_config);

    pcnt_counter_pause(_pcnt_unit);
    pcnt_counter_clear(_pcnt_unit);
    pcnt_counter_resume(_pcnt_unit);
}

void Motor::set_duty(float u) {
    if (u > 0) {
        gpio_set_level(_in1Pin, 1);
        gpio_set_level(_in2Pin, 0);
    } else if (u < 0) {
        gpio_set_level(_in1Pin, 0);
        gpio_set_level(_in2Pin, 1);
        u = -u;
    } else {
        // u == 0 → parar o motor sem frenagem
        gpio_set_level(_in1Pin, 0);
        gpio_set_level(_in2Pin, 0);
    }

    u = std::fmin(u, 8191); // Já garantido que é módulo positivo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, static_cast<uint32_t>(u));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel);

    if (_verbose) {
        ESP_LOGI(TAG, "[%s] set_duty: %.2f", _nome.c_str(), u);
    }
}


void Motor::update_velocity() {
    pcnt_get_counter_value(_pcnt_unit, &_curr_pulses);  // ✅ Corrigido tipo para int16_t
    int delta = _curr_pulses - _prev_pulses;
    _prev_pulses = _curr_pulses;

    float rotacoes = static_cast<float>(delta) / (_pulsos_por_rotacao * 2); // quadratura x2
    _velocity = rotacoes * 2.0f * M_PI / 0.1f; // rad/s em 100ms

    if (_verbose) {
        ESP_LOGI(TAG, "[%s] Pulsos: %d Velocidade: %.2f rad/s", _nome.c_str(), delta, _velocity);
    }
}

float Motor::get_velocity() const {
    return _velocity;
}

void Motor::set_verbose(bool v) {
    _verbose = v;
}
