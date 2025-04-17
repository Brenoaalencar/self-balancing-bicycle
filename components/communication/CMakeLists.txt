idf_component_register(
    SRCS "communication.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash esp_netif spiffs esp_http_server json
)
