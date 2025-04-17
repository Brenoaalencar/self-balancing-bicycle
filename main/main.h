#ifndef main_H
#define main_H

//Default libs (C, IDF, etc)
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>
#include "esp_camera.h"
#include "driver/gpio.h"


//Custom libs
//#include "../components/i2c_bus.h"
//#include "../components/imu.h"
//#include "../components/servo_m.c"
#include "motor.hpp"
#include "imu.hpp"
#include "communication.hpp"


//GPIOs define
#define SERVO_YAW_PIN       GPIO_NUM_33
#define SERVO_PITCH_PIN     GPIO_NUM_6

//Objects
//NVS nvs;
//LOGGER logger;
//extern I2C i2c_bus;

//Functions


#endif
