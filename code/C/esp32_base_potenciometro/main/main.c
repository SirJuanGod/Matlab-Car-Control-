#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

#include "protocol_defs.h"
#include "adc_reader.h"
#include "uart_matlab.h"
#include "espnow_base.h"

static const char *TAG = "MAIN_BASE";

#define LOOP_PERIOD_MS 20 // 50 Hz tasa de transmision hacia Carro y MATLAB

static QueueHandle_t s_adc_angle_queue = NULL;
static QueueHandle_t s_car_telemetry_queue = NULL;
static QueueHandle_t s_matlab_cmd_queue = NULL;

static void base_hub_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);

    uint32_t seq_count = 0;
    float current_theta = 0.0f;
    car_to_base_packet_t last_car_tel;
    bool has_car_tel = false;

    // Valores por defecto de consigna
    base_to_car_packet_t cmd_to_car = {
        .seq_num = 0,
        .theta_meas_deg = 0.0f,
        .theta_ref_deg = 35.0f, // Consigna nominal segun init_parametros.m
        .d_ref_cm = 10.0f,      // Distancia nominal d0 = 10 cm
        .matlab_pwm_feed = 0.0f,
        .control_mode = 0,      // Control autonomo en carro por defecto
        .enable_motors = 1,
        .enable_fan = 1,
        .flags = 0,
        .checksum = 0
    };

    memset(&last_car_tel, 0, sizeof(last_car_tel));

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Obtener la medicion angular mas reciente del ADC
        if (xQueueReceive(s_adc_angle_queue, &current_theta, 0) == pdTRUE) {
            cmd_to_car.theta_meas_deg = current_theta;
        } else {
            cmd_to_car.theta_meas_deg = adc_reader_get_last_angle();
        }

        // 2. Verificar si llegaron comandos nuevos desde MATLAB
        base_to_car_packet_t new_cmd;
        if (xQueueReceive(s_matlab_cmd_queue, &new_cmd, 0) == pdTRUE) {
            cmd_to_car.theta_ref_deg = new_cmd.theta_ref_deg;
            cmd_to_car.d_ref_cm = new_cmd.d_ref_cm;
            cmd_to_car.matlab_pwm_feed = new_cmd.matlab_pwm_feed;
            cmd_to_car.control_mode = new_cmd.control_mode;
            cmd_to_car.enable_motors = new_cmd.enable_motors;
            cmd_to_car.enable_fan = new_cmd.enable_fan;
        }

        // 3. Verificar si llego telemetria nueva del Carro por ESP-NOW
        if (xQueueReceive(s_car_telemetry_queue, &last_car_tel, 0) == pdTRUE) {
            has_car_tel = true;
        }

        // 4. Enviar datos formateados hacia la interfaz de MATLAB por Serial
        uart_matlab_send_telemetry(has_car_tel ? &last_car_tel : NULL, cmd_to_car.theta_meas_deg);

        // 5. Enviar paquete de control y medicion hacia el Carro por ESP-NOW
        cmd_to_car.seq_num = seq_count++;
        espnow_base_send(&cmd_to_car);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Iniciando ESP32 Estacion Base (ADC + MATLAB + ESP-NOW) ===");

    // 1. Crear colas de comunicacion entre tareas (no bloqueantes)
    s_adc_angle_queue = xQueueCreate(1, sizeof(float));
    s_car_telemetry_queue = xQueueCreate(1, sizeof(car_to_base_packet_t));
    s_matlab_cmd_queue = xQueueCreate(1, sizeof(base_to_car_packet_t));

    configASSERT(s_adc_angle_queue);
    configASSERT(s_car_telemetry_queue);
    configASSERT(s_matlab_cmd_queue);

    // 2. Inicializar subsistemas
    ESP_ERROR_CHECK(adc_reader_init(s_adc_angle_queue));
    ESP_ERROR_CHECK(uart_matlab_init(s_matlab_cmd_queue));
    ESP_ERROR_CHECK(espnow_base_init(s_car_telemetry_queue, NULL)); // Broadcast MAC

    // 3. Iniciar temporizador no bloqueante de muestreo ADC
    ESP_ERROR_CHECK(adc_reader_start());

    // 4. Crear tarea principal del Hub de Comunicacion
    xTaskCreatePinnedToCore(base_hub_task, "base_hub_task", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Sistema de Estacion Base operativo al 100%%.");
}
