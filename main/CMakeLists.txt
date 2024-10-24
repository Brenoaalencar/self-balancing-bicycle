idf_component_register(SRCS "testcamera.cpp" 
                    INCLUDE_DIRS "."
                    REQUIRES esp_system
                    REQUIRES driver
                    REQUIRES esp_timer
                    REQUIRES esp32-camera
                    PRIV_REQUIRES nvs_flash)

                    