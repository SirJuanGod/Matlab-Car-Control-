#ifndef ESPNOW_CAR_H
#define ESPNOW_CAR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol_defs.h"

/**
 * @brief Inicializa WiFi en modo STA y el protocolo ESP-NOW en el nodo Carro.
 * 
 * @param rx_cmd_queue Cola donde se entregaran los paquetes recibidos desde la Base.
 * @param target_base_mac Direccion MAC de la Estacion Base (o NULL para Broadcast).
 * @return esp_err_t ESP_OK en caso de exito.
 */
esp_err_t espnow_car_init(QueueHandle_t rx_cmd_queue, const uint8_t *target_base_mac);

/**
 * @brief Envia el paquete de telemetria completa del carro hacia la Base.
 * 
 * @param telemetry Puntero a la telemetria calculada.
 * @return esp_err_t ESP_OK si se encoló correctamente en el buffer de radio.
 */
esp_err_t espnow_car_send_telemetry(car_to_base_packet_t *telemetry);

#endif // ESPNOW_CAR_H
