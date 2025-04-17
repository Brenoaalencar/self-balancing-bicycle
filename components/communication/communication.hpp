#pragma once

#include "esp_http_server.h"

class Communication {
public:
    Communication();
    void wifi();
    void begin();
    void set_setpoint(double alfa_d, double y_d);
    void get_setpoint(double &alfa_d, double &y_d);
    static void task(void *param);

private:
    double alfa_d;
    double y_d;
    httpd_handle_t server;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};
