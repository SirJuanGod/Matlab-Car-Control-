#ifndef ADC_READER_H
#define ADC_READER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define POT_ADC_CHANNEL        ADC_CHANNEL_6 // GPIO34 en ESP32 ADC1
#define ADC_MAX_COUNTS         4095.0f
#define ANGLE_MAX_DEG          180.0f
#define ADC_SAMPLING_PERIOD_US 10000 // 10 ms = 100 Hz

/**
 * @brief Inicializa el periferico ADC1 en GPIO34 y configura el muestreo periodico no bloqueante.
 * 
 * @param out_queue Cola FreeRTOS donde se depositaran las lecturas de angulo (float).
 * @return esp_err_t ESP_OK si la inicializacion fue exitosa.
 */
esp_err_t adc_reader_init(QueueHandle_t out_queue);

/**
 * @brief Inicia el temporizador de muestreo no bloqueante del ADC.
 */
esp_err_t adc_reader_start(void);

/**
 * @brief Detiene el temporizador de muestreo.
 */
esp_err_t adc_reader_stop(void);

/**
 * @brief Obtiene la ultima lectura angular calculada [0.0 - 180.0 deg].
 */
float adc_reader_get_last_angle(void);

#endif // ADC_READER_H
