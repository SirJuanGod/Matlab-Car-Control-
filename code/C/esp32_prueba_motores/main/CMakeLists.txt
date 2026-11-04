idf_component_register(SRCS "main.c"
                            "../../esp32_carro_control/main/mpu6050.c"
                       INCLUDE_DIRS "." "../../esp32_carro_control/main"
                       REQUIRES esp_driver_gpio esp_driver_ledc driver esp_driver_i2c)
