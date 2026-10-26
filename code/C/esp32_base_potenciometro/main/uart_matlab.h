#ifndef UART_MATLAB_H
#define UART_MATLAB_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol_defs.h"

#define UART_MATLAB_PORT      UART_NUM_0
#define UART_MATLAB_BAUDRATE  115200
#define UART_MATLAB_TX_PIN    1
#define UART_MATLAB_RX_PIN    3
#define UART_BUF_SIZE         1024

/**
 * @brief Inicializa el puerto UART0 con buffers de recepcion y colas no bloqueantes.
 * 
 * @param cmd_out_queue Cola donde se publicaran los comandos parseados recibidos de MATLAB.
 * @return esp_err_t ESP_OK en caso de exito.
 */
esp_err_t uart_matlab_init(QueueHandle_t cmd_out_queue);

/**
 * @brief Envia una linea de telemetria formateada por UART hacia MATLAB.
 * 
 * Formato: $TEL,<theta_meas>,<dist_est>,<vel_est>,<ax>,<u_fan>,<u_car>,<error_d>,<status>\n
 * 
 * @param telemetry Datos recibidos del carro.
 * @param theta_meas Angulo actual medido por el ADC de la compuerta.
 */
void uart_matlab_send_telemetry(const car_to_base_packet_t *telemetry, float theta_meas);

#endif // UART_MATLAB_H
