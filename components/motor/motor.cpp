#include "motor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

static const char* TAG = "Motor";

Motor::Motor(gpio_num_t pinA, gpio_num_t pinB, int pulsos_por_rotacao, const std::string& nome,
             gpio_num_t pwmPin, ledc_channel_t pwmChannel, ledc_timer_t pwmTimer,
             gpio_num_t in1Pin, gpio_num_t in2Pin)
    : _pinA(pinA), _pinB(pinB), _pulsos_por_rotacao(pulsos_por_rotacao), _nome(nome),
      _pwmPin(pwmPin), _pwmChannel(pwmChannel), _pwmTimer(pwmTimer),
      _in1Pin(in1Pin), _in2Pin(in2Pin),
      _last_time(esp_timer_get_time())
{}

void Motor::begin() {
    // Direção
    gpio_config_t dir_conf = {};
    dir_conf.mode = GPIO_MODE_OUTPUT;
    dir_conf.pin_bit_mask = (1ULL << _in1Pin) | (1ULL << _in2Pin);
    gpio_config(&dir_conf);
    gpio_set_level(_in1Pin, 0);
    gpio_set_level(_in2Pin, 0);

    // PWM
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

    // Canal 0
    {
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = _pinA;
        cfg.ctrl_gpio_num  = _pinB;
        cfg.channel        = PCNT_CHANNEL_0;
        cfg.unit           = _pcnt_unit;
        cfg.pos_mode       = PCNT_COUNT_INC;
        cfg.neg_mode       = PCNT_COUNT_DEC;
        cfg.lctrl_mode     = PCNT_MODE_KEEP;
        cfg.hctrl_mode     = PCNT_MODE_REVERSE;
        cfg.counter_h_lim  = 32000;
        cfg.counter_l_lim  = -32000;
        pcnt_unit_config(&cfg);
    }

    // Canal 1
    {
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = _pinB;
        cfg.ctrl_gpio_num  = _pinA;
        cfg.channel        = PCNT_CHANNEL_1;
        cfg.unit           = _pcnt_unit;
        cfg.pos_mode       = PCNT_COUNT_INC;
        cfg.neg_mode       = PCNT_COUNT_DEC;
        cfg.lctrl_mode     = PCNT_MODE_REVERSE;
        cfg.hctrl_mode     = PCNT_MODE_KEEP;
        cfg.counter_h_lim  = 32000;
        cfg.counter_l_lim  = -32000;
        pcnt_unit_config(&cfg);
    }

    pcnt_counter_pause(_pcnt_unit);
    pcnt_counter_clear(_pcnt_unit);
    pcnt_counter_resume(_pcnt_unit);

    _last_time = esp_timer_get_time();
    _velocity = 0;
    _velocity_raw = 0;
    _total_pulses = 0;
    _prev_pulses = 0;
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
        gpio_set_level(_in1Pin, 0);
        gpio_set_level(_in2Pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel);
        return;
    }

    u = std::fmin(u, 1.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, static_cast<uint32_t>(8192 * u));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel);

    if (_verbose) {
        ESP_LOGI(TAG, "[%s] set_duty: %.2f", _nome.c_str(), u);
    }
}

void Motor::update_velocity() {
    int16_t pulses = 0;
    pcnt_get_counter_value(_pcnt_unit, &pulses);

    int delta = pulses - _prev_pulses;

    // Previne overflow
    if (std::abs(pulses) > 31000) {
        pcnt_counter_clear(_pcnt_unit);
        _prev_pulses = 0;
    } else {
        _prev_pulses = pulses;
    }

    _total_pulses += delta;

    int64_t now = esp_timer_get_time();  // microssegundos
    float dt = (now - _last_time) / 1e6f;  // segundos
    _last_time = now;

    float rotacoes = static_cast<float>(delta) / _pulsos_por_rotacao;
    _velocity_raw = rotacoes * 2.0f * M_PI / dt;

    // Filtro passa-baixa: suaviza ruído
    const float alpha = 0.3f; // suavização (0=suave, 1=bruto)
    _velocity = alpha * _velocity_raw + (1.0f - alpha) * _velocity;
    
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
