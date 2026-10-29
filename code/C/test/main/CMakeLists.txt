# Corre test_control.c en un ESP32 (real o QEMU) cuando no hay gcc para PC.
idf_component_register(SRCS "../test_control.c"
                            "../../esp32_carro_control/main/car_controller.c"
                            "../../esp32_carro_control/main/lqr_simo.c"
                            "../../esp32_carro_control/main/kalman_observer.c"
                       INCLUDE_DIRS "../../esp32_carro_control/main")
