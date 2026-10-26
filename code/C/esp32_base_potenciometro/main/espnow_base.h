#ifndef ESPNOW_BASE_H
#define ESPNOW_BASE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol_defs.h"

/**
 * @brief Inicializa WiFi en modo STA y el protocolo ESP-NOW para la Estación Base.
 * 
 * @param rx_telemetry_queue Cola donde se entregaran los paquetes recibidos del Carro.
 * @param target_peer_mac Direccion MAC del Carro (o NULL para usar Broadcast FF:FF:FF:FF:FF:FF).
 * @return esp_err_t ESP_OK en caso de exito.
 */
esp_err_t espnow_base_init(QueueHandle_t rx_telemetry_queue, const uint8_t *target_peer_mac);

/**
 * @brief Envia el paquete de consignas y angulo medido hacia el Carro.
 * 
 * @param packet Puntero al paquete a enviar.
 * @return esp_err_t ESP_OK si la transmision se encoló correctamente.
 */
esp_err_t espnow_base_send(base_to_car_packet_t *packet);

#endif // ESPNOW_BASE_H
