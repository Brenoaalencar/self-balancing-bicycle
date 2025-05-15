idf_component_register(
    SRCS "imu.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash
)
