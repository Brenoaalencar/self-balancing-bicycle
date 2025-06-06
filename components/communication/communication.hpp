#pragma once

#include "esp_http_server.h"
#include <array>

struct TelemetryData {
    float teta;
    float teta_d;
    float alfa;
    float alfa_d;
    float y_d;
    float e_alpha_i;
    float e_y_i;
    float u_vl;
    float u_vr;
};

class Communication {
public:
    Communication();
    void wifi();
    void begin();
    void set_setpoint(double alfa_d, double y_d);
    void get_setpoint(double &alfa_d, double &y_d);
    void set_telemetry(const TelemetryData& data);
    static void task(void *param);

private:
    double alfa_d;
    double y_d;
    TelemetryData telemetry_data;
    httpd_handle_t server;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
};
