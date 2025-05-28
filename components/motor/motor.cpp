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
    // direção
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
    
    // PCNT: quadratura ×4
    _pcnt_unit = (_pwmChannel == LEDC_CHANNEL_0) ? PCNT_UNIT_0 : PCNT_UNIT_1;

    // Canal 0: bordas de A
    {
      pcnt_config_t cfg = {};
      cfg.pulse_gpio_num = _pinA;       // sinal A
      cfg.ctrl_gpio_num  = _pinB;       // sinal B
      cfg.channel        = PCNT_CHANNEL_0;
      cfg.unit           = _pcnt_unit;
      cfg.pos_mode       = PCNT_COUNT_INC;    // A sobe → +1
      cfg.neg_mode       = PCNT_COUNT_DEC;    // A desce → -1
      cfg.lctrl_mode     = PCNT_MODE_KEEP;    // B=0 mantém
      cfg.hctrl_mode     = PCNT_MODE_REVERSE; // B=1 inverte
      cfg.counter_h_lim  = 32000;
      cfg.counter_l_lim  = -32000;
      pcnt_unit_config(&cfg);
    }

    // Canal 1: bordas de B
    {
      pcnt_config_t cfg = {};
      cfg.pulse_gpio_num = _pinB;       // sinal B
      cfg.ctrl_gpio_num  = _pinA;       // sinal A
      cfg.channel        = PCNT_CHANNEL_1;
      cfg.unit           = _pcnt_unit;
      cfg.pos_mode       = PCNT_COUNT_INC;    // B sobe → +1
      cfg.neg_mode       = PCNT_COUNT_DEC;    // B desce → -1
      cfg.lctrl_mode     = PCNT_MODE_REVERSE; // A=0 inverte
      cfg.hctrl_mode     = PCNT_MODE_KEEP;    // A=1 mantém
      cfg.counter_h_lim  = 32000;
      cfg.counter_l_lim  = -32000;
      pcnt_unit_config(&cfg);
    }

    // iniciar
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
        gpio_set_level(_in1Pin, 0);
        gpio_set_level(_in2Pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel);
    }

    u = std::fmin(u, 1.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, static_cast<uint32_t>(8182 * u));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel);

    if (_verbose) {
        ESP_LOGI(TAG, "[%s] set_duty: %.2f", _nome.c_str(), u);
    }
}

void Motor::update_velocity() {
    pcnt_get_counter_value(_pcnt_unit, &_curr_pulses);
    int delta = _curr_pulses - _prev_pulses;

    // Detecta aproximação dos limites → limpa para evitar overflow
    if (std::abs(_curr_pulses) > 31000) {
        pcnt_counter_clear(_pcnt_unit);
        _prev_pulses = 0;
        _curr_pulses = 0;
        delta = 0;  // descarta leitura atual
    } else {
        _prev_pulses = _curr_pulses;
    }

    float rotacoes = static_cast<float>(delta) / (_pulsos_por_rotacao * 8); // quadratura ×4
    _velocity = rotacoes * 2.0f * M_PI / 0.01f; // rad/s (10ms de intervalo)

    if (_verbose) {
        ESP_LOGI(TAG, "[%s] Pulsos: %d Velocidade: %.2f rad/s",
                 _nome.c_str(), delta, _velocity);
    }
}

float Motor::get_velocity() const {
    return _velocity;
}

void Motor::set_verbose(bool v) {
    _verbose = v;
}
