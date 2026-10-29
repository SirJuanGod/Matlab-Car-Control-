#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

#include "protocol_defs.h"
#include "mpu6050.h"
#include "kalman_observer.h"
#include "motor_fan_pwm.h"
#include "car_controller.h"
#include "espnow_car.h"

static const char *TAG = "MAIN_CAR";

#define CONTROL_LOOP_PERIOD_MS 10 // 100 Hz (dt = 0.01 s) identico a imu_Ts en MATLAB
#define TELEMETRY_PERIOD_MS    20 // 50 Hz envio de paquetes hacia la base

static QueueHandle_t s_rx_cmd_queue = NULL;
static QueueHandle_t s_tx_telemetry_queue = NULL;

static void car_control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);
    const float dt_s = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;

    uint32_t seq_count = 0;
    float current_ax = 0.0f;
    float last_u_motor = 0.0f;
    uint32_t comm_loss_ticks = 0;
    bool link_established = false;

    // Comandos vigentes (arranque seguro: actuadores deshabilitados hasta conectar con la base)
    base_to_car_packet_t active_cmd = {
        .seq_num = 0,
        .theta_meas_deg = 0.0f,
        .theta_ref_deg = 35.0f,
        .enable_motors = 0,     // Inicia seguro (deshabilitado)
        .enable_fan = 0,        // Inicia seguro (deshabilitado)
        .checksum = 0
    };

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Recepcion de consignas con Failsafe de seguridad por perdida de comunicacion
        base_to_car_packet_t incoming_cmd;
        if (xQueueReceive(s_rx_cmd_queue, &incoming_cmd, 0) == pdTRUE) {
            active_cmd = incoming_cmd;
            comm_loss_ticks = 0;
            link_established = true;
        } else if (link_established) {
            comm_loss_ticks++;
            // Failsafe fisico: Si pasan > 500 ms (50 ciclos) sin paquetes de la base,
            // se apagan motores y ventilador de inmediato para proteger el montaje mecanico.
            if (comm_loss_ticks > 50) {
                active_cmd.enable_motors = 0;
                active_cmd.enable_fan = 0;
            }
        }

        // 2. Leer acelerometro MPU6050 por I2C (no bloqueante)
        if (mpu6050_read_accel_x(&current_ax) != ESP_OK) {
            current_ax = 0.0f;
        }

        // 3. Actualizar Observador de Estados Kalman (estimacion de distancia y velocidad)
        kalman_state_t k_state = kalman_observer_update(current_ax, last_u_motor, dt_s);

        // Motores solo con IMU operativa: sin ella el observador estima a ciegas
        bool motors_on = (active_cmd.enable_motors != 0) && mpu6050_is_ready();
        bool fan_on = (active_cmd.enable_fan != 0);

        // 4. Control: LQR+I SIMO (ventilador y d_ref) + PID de distancia del carro
        controller_output_t ctrl_out = car_controller_step(
            k_state.d_est_cm,
            k_state.v_est,
            active_cmd.theta_meas_deg,
            active_cmd.theta_ref_deg,
            fan_on,
            motors_on,
            dt_s
        );

        // El observador debe ver el esfuerzo realmente aplicado, no el calculado
        last_u_motor = motors_on ? ctrl_out.u_car_percent : 0.0f;

        // 5. Aplicar senales fisicas a los actuadores
        fan_set_pwm_percent(ctrl_out.u_fan_percent, fan_on);
        car_set_motors(ctrl_out.u_car_percent, motors_on);

        // 6. Preparar paquete de telemetria para el enlace ESP-NOW
        car_to_base_packet_t tel_pkt = {
            .seq_num = seq_count++,
            .dist_est_cm = k_state.d_est_cm,
            .vel_est_ms = k_state.v_est,
            .accel_x_ms2 = current_ax,
            .u_fan_pwm = ctrl_out.u_fan_percent,
            .u_motor_pwm = ctrl_out.u_car_percent,
            .error_dist_cm = ctrl_out.error_dist_cm,
            .imu_ready = mpu6050_is_ready() ? 1 : 0,
            .control_status = (fan_on ? 1 : 0) | (motors_on ? 2 : 0),
            .checksum = 0
        };

        if (s_tx_telemetry_queue != NULL) {
            xQueueOverwrite(s_tx_telemetry_queue, &tel_pkt);
        }
    }
}

static void car_telemetry_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

    car_to_base_packet_t pkt_to_send;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xQueueReceive(s_tx_telemetry_queue, &pkt_to_send, 0) == pdTRUE) {
            espnow_car_send_telemetry(&pkt_to_send);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Iniciando ESP32 Carro (IMU + Kalman + Control + ESP-NOW) ===");

    // 1. Crear colas FreeRTOS
    s_rx_cmd_queue = xQueueCreate(1, sizeof(base_to_car_packet_t));
    s_tx_telemetry_queue = xQueueCreate(1, sizeof(car_to_base_packet_t));

    configASSERT(s_rx_cmd_queue);
    configASSERT(s_tx_telemetry_queue);

    // 2. Inicializar perifericos y hardware
    ESP_ERROR_CHECK(motor_fan_pwm_init());
    actuators_emergency_stop(); // Seguridad: arrancar apagado

    if (mpu6050_init() == ESP_OK) {
        // Calibracion de sesgo de reposo (100 muestras)
        mpu6050_calibrate_bias(100);
    } else {
        ESP_LOGE(TAG, "Fallo al inicializar MPU6050. Verifique cables I2C SDA/SCL.");
    }

    kalman_observer_init();
    car_controller_init(0.0f);

    // 3. Inicializar ESP-NOW
    static const uint8_t peer_mac[6] = BASE_MAC_ADDR;
    ESP_ERROR_CHECK(espnow_car_init(s_rx_cmd_queue, peer_mac));

    // 4. Crear tareas FreeRTOS
    // Tarea de control a 100 Hz en Nucleo 1
    xTaskCreatePinnedToCore(car_control_task, "car_ctrl_task", 4096, NULL, 6, NULL, 1);

    // Tarea de despacho de telemetria a 50 Hz en Nucleo 0
    xTaskCreatePinnedToCore(car_telemetry_task, "car_tx_task", 3072, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Sistema de Control del Carro en ejecucion.");
}
