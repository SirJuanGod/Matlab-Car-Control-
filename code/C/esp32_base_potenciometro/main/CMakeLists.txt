idf_component_register(SRCS "main.c"
                            "adc_reader.c"
                            "uart_matlab.c"
                            "espnow_base.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash driver esp_timer esp_adc esp_driver_uart esp_driver_gpio)
