idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_system driver esp_timer esp32-camera motor imu communication
    PRIV_REQUIRES nvs_flash
)
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)