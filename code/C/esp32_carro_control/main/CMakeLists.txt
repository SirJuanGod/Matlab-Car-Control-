idf_component_register(SRCS "main.c"
                            "mpu6050.c"
                            "kalman_observer.c"
                            "motor_fan_pwm.c"
                            "car_controller.c"
                            "lqr_simo.c"
                            "espnow_car.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash driver esp_driver_gpio esp_driver_ledc esp_driver_i2c)
