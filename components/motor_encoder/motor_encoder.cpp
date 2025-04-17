#include "motor_encoder.hpp"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cmath>

MotorEncoder::MotorEncoder(gpio_num_t ch1_pin, gpio_num_t ch2_pin, uint8_t num_dentes, const char* nome)
    : ch1(ch1_pin), ch2(ch2_pin), dentes(num_dentes), name(nome),
      contador1(0), contador2(0), sentido(true), velocity(0), verbose(false) {}

void MotorEncoder::begin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ch1) | (1ULL << ch2);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0); // uma única vez no sistema
    gpio_isr_handler_add(ch1, isr_ch1, this);
    gpio_isr_handler_add(ch2, isr_ch2, this);

    xTaskCreatePinnedToCore(task_velocity, name, 2048, this, 1, nullptr, 1);
}

void IRAM_ATTR MotorEncoder::isr_ch1(void* arg) {
    auto* self = static_cast<MotorEncoder*>(arg);
    self->contador1++;
}

void IRAM_ATTR MotorEncoder::isr_ch2(void* arg) {
    auto* self = static_cast<MotorEncoder*>(arg);
    self->contador2++;

    static bool last = false;
    bool curr = gpio_get_level(self->ch2);
    if (!last && curr) {
        self->sentido = gpio_get_level(self->ch1) == 0;
    }
    last = curr;
}

void MotorEncoder::task_velocity(void* pvParams) {
    auto* self = static_cast<MotorEncoder*>(pvParams);
    int64_t last_c1 = 0, last_c2 = 0;
    int64_t last_time_us = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));

        int64_t now_c1 = self->contador1;
        int64_t now_c2 = self->contador2;
        int64_t delta = ((now_c1 - last_c1) + (now_c2 - last_c2)) / 2;
        last_c1 = now_c1;
        last_c2 = now_c2;

        int64_t now_time_us = esp_timer_get_time();
        float dt = (now_time_us - last_time_us) / 1e6;
        last_time_us = now_time_us;

        int pulsos_por_rotacao = self->dentes * 2;
        float rotacoes = (float)delta / pulsos_por_rotacao;
        float vel = (rotacoes * RAD_PER_ROTATION) / dt;
        if (!self->sentido) vel = -vel;

        self->velocity = vel;

        if (self->verbose) {
            ESP_LOGI(self->name, "Velocidade: %.2f rad/s", vel);
        }
    }
}

float MotorEncoder::get_velocity() const {
    return velocity;
}

void MotorEncoder::set_verbose(bool v) {
    verbose = v;
}
