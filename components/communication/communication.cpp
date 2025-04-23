#include "communication.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <esp_spiffs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

#define WIFI_SSID ""
#define WIFI_PASS ""
static const char *TAG = "Communication";

static Communication *singleton = nullptr;

// HTML handler
static esp_err_t html_handler(httpd_req_t *req) {
    FILE *f = fopen("/storage/joystick.html", "r");
    if (!f) {
        ESP_LOGE(TAG, "Erro ao abrir joystick.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        httpd_resp_sendstr_chunk(req, line);
    }
    fclose(f);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// JSON POST handler
static esp_err_t json_post_handler(httpd_req_t *req) {
    char buf[256];
    int len = req->content_len;
    int recv_len = 0;

    while (recv_len < len) {
        int ret = httpd_req_recv(req, buf + recv_len, len - recv_len);
        if (ret <= 0) return ESP_FAIL;
        recv_len += ret;
    }
    buf[recv_len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    cJSON *cmd = cJSON_GetObjectItem(json, "command");
    cJSON *val = cJSON_GetObjectItem(json, "value");

    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "movement") == 0 &&
        cJSON_IsArray(val) && cJSON_GetArraySize(val) == 2) {

        double alfa = cJSON_GetArrayItem(val, 0)->valuedouble;
        double y = cJSON_GetArrayItem(val, 1)->valuedouble;
        singleton->set_setpoint(alfa, y);

        ESP_LOGI(TAG, "Comando: alfa=%.2f, y=%.2f", alfa, y);
        httpd_resp_sendstr(req, "OK");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
    }

    cJSON_Delete(json);
    return ESP_OK;
}

// Constructor
Communication::Communication() : alfa_d(0), y_d(0), server(nullptr) {
    singleton = this;
}

void Communication::wifi() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

void Communication::begin() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/storage",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t result = esp_vfs_spiffs_register(&conf);

    if (result != ESP_OK){
        ESP_LOGE(__func__,"Faild to initialize SPIFFS (%s)",esp_err_to_name(result));
        return;
    }

    wifi();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t html_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = html_handler
        };
        httpd_register_uri_handler(server, &html_uri);

        httpd_uri_t json_uri = {
            .uri = "/movement",
            .method = HTTP_POST,
            .handler = json_post_handler
        };
        httpd_register_uri_handler(server, &json_uri);
    }
}

void Communication::set_setpoint(double alfa, double y) {
    portENTER_CRITICAL(&mux);
    alfa_d = alfa;
    y_d = y;
    portEXIT_CRITICAL(&mux);
}

void Communication::get_setpoint(double &a, double &y) {
    portENTER_CRITICAL(&mux);
    a = alfa_d;
    y = y_d;
    portEXIT_CRITICAL(&mux);
}

void Communication::task(void *param) {
    Communication *self = static_cast<Communication*>(param);
    nvs_flash_init();
    self->begin();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
