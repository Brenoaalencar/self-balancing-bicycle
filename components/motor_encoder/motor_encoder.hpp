#pragma once

#include "driver/gpio.h"
#include "esp_attr.h"

class MotorEncoder {
public:
    MotorEncoder(gpio_num_t ch1_pin, gpio_num_t ch2_pin, uint8_t num_dentes, const char* nome);

    void begin();
    float get_velocity() const;
    void set_verbose(bool v);

    static void IRAM_ATTR isr_ch1(void* arg);
    static void IRAM_ATTR isr_ch2(void* arg);

private:
    static constexpr int PERIOD_MS = 100;
    static constexpr float RAD_PER_ROTATION = 6.28319f;

    gpio_num_t ch1, ch2;
    uint8_t dentes;
    const char* name;

    volatile int64_t contador1;
    volatile int64_t contador2;
    volatile bool sentido;
    float velocity;
    bool verbose;

    static void task_velocity(void* pvParams);
};
