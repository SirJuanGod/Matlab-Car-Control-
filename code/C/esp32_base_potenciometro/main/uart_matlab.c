#include "uart_matlab.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "UART_MATLAB";

static QueueHandle_t s_uart_event_queue = NULL;
static QueueHandle_t s_cmd_out_queue = NULL;

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    static uint8_t dtmp[UART_BUF_SIZE]; // estatico: no cabe en el stack de la tarea
    char line_buffer[128];
    size_t line_idx = 0;

    while (1) {
        if (xQueueReceive(s_uart_event_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    size_t to_read = (event.size < sizeof(dtmp)) ? event.size : sizeof(dtmp);
                    int len = uart_read_bytes(UART_MATLAB_PORT, dtmp, to_read, portMAX_DELAY);
                    for (int i = 0; i < len; i++) {
                        char c = (char)dtmp[i];
                        if (c == '\n' || c == '\r') {
                            if (line_idx > 0) {
                                line_buffer[line_idx] = '\0';
                                line_idx = 0;

                                // Parsear comando de MATLAB: $CMD,<theta_ref>,<en_motors>,<en_fan>
                                float th_ref = 0.0f;
                                int en_m = 0, en_f = 0;

                                if (strncmp(line_buffer, "$CMD,", 5) == 0) {
                                    int parsed = sscanf(line_buffer + 5, "%f,%d,%d", &th_ref, &en_m, &en_f);
                                    // Trama completa obligatoria: campos faltantes no deben re-habilitar actuadores.
                                    // Consigna dentro del rango mecanico [0, 90] deg (isfinite descarta "nan").
                                    if (parsed == 3 && isfinite(th_ref) && th_ref >= 0.0f && th_ref <= 90.0f) {
                                        base_to_car_packet_t cmd;
                                        memset(&cmd, 0, sizeof(cmd));
                                        cmd.theta_ref_deg = th_ref;
                                        cmd.enable_motors = (uint8_t)(en_m != 0);
                                        cmd.enable_fan = (uint8_t)(en_f != 0);

                                        if (s_cmd_out_queue != NULL) {
                                            xQueueOverwrite(s_cmd_out_queue, &cmd);
                                        }
                                        ESP_LOGD(TAG, "CMD recibido: th_ref=%.1f, motores=%d, ventilador=%d", th_ref, en_m, en_f);
                                    }
                                }
                            }
                        } else {
                            if (line_idx < sizeof(line_buffer) - 1) {
                                line_buffer[line_idx++] = c;
                            } else {
                                line_idx = 0; // Desborde, reiniciar linea
                            }
                        }
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART HW FIFO desbordado");
                    uart_flush_input(UART_MATLAB_PORT);
                    xQueueReset(s_uart_event_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART Buffer lleno");
                    uart_flush_input(UART_MATLAB_PORT);
                    xQueueReset(s_uart_event_queue);
                    break;
                default:
                    break;
            }
        }
    }
}

esp_err_t uart_matlab_init(QueueHandle_t cmd_out_queue) {
    s_cmd_out_queue = cmd_out_queue;

    uart_config_t uart_config = {
        .baud_rate = UART_MATLAB_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_MATLAB_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_MATLAB_PORT, UART_MATLAB_TX_PIN, UART_MATLAB_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_MATLAB_PORT, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 20, &s_uart_event_queue, 0));

    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 4096, NULL, 4, NULL, 0); // sscanf(%f) usa bastante pila

    ESP_LOGI(TAG, "UART MATLAB inicializado a %d baudios.", UART_MATLAB_BAUDRATE);
    return ESP_OK;
}

void uart_matlab_send_telemetry(const car_to_base_packet_t *telemetry, float theta_meas) {
    char tx_buf[160];
    int len;

    if (telemetry != NULL) {
        len = snprintf(tx_buf, sizeof(tx_buf),
                       "$TEL,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%u\n",
                       theta_meas,
                       telemetry->dist_est_cm,
                       telemetry->vel_est_ms,
                       telemetry->accel_x_ms2,
                       telemetry->u_fan_pwm,
                       telemetry->u_motor_pwm,
                       telemetry->error_dist_cm,
                       telemetry->control_status);
    } else {
        len = snprintf(tx_buf, sizeof(tx_buf),
                       "$TEL,%.2f,0.00,0.000,0.000,0.00,0.00,0.00,0\n",
                       theta_meas);
    }

    // snprintf devuelve el largo sin truncar: no leer mas alla de tx_buf
    if (len >= (int)sizeof(tx_buf)) len = sizeof(tx_buf) - 1;
    if (len > 0) {
        uart_write_bytes(UART_MATLAB_PORT, tx_buf, len);
    }
}
