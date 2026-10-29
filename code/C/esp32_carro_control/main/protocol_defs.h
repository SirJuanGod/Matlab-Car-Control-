#ifndef PROTOCOL_DEFS_H
#define PROTOCOL_DEFS_H

#include <stdint.h>
#include <stddef.h>

#define ESPNOW_WIFI_CHANNEL 1

// MAC STA de cada nodo (cada ESP32 imprime la suya al arrancar).
// Con FF:FF:FF:FF:FF:FF se usa broadcast SIN filtro: cualquier ESP32 en el canal puede mandar comandos.
#define BASE_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define CAR_MAC_ADDR  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#pragma pack(push, 1)

/**
 * @brief Paquete transmitido desde la Estación Base hacia el Carro.
 */
typedef struct {
    uint32_t seq_num;          // Contador secuencial de trama
    float    theta_meas_deg;   // Angulo medido de la compuerta por ADC [0.0 - 90.0 deg]
    float    theta_ref_deg;    // Consigna de angulo deseada [deg] (d_ref la calcula el LQR del carro)
    uint8_t  enable_motors;    // 1: Habilitar motores de traccion, 0: Parada segura
    uint8_t  enable_fan;       // 1: Habilitar ventilador, 0: Ventilador apagado
    uint16_t checksum;         // Suma de comprobacion
} base_to_car_packet_t;

/**
 * @brief Paquete de telemetria transmitido desde el Carro hacia la Estación Base.
 */
typedef struct {
    uint32_t seq_num;          // Contador secuencial de trama
    float    dist_est_cm;      // Distancia estimada d(t) = 10cm - Delta_x [cm]
    float    vel_est_ms;       // Velocidad lineal estimada por Kalman [m/s]
    float    accel_x_ms2;      // Aceleracion longitudinal ax medida por IMU [m/s^2]
    float    u_fan_pwm;        // Ciclo de trabajo aplicado al ventilador [10.0 - 90.0 %]
    float    u_motor_pwm;      // Esfuerzo de traccion del PID del carro [-100.0 a +100.0 %]
    float    error_dist_cm;    // Error de distancia instantaneo e_d [cm]
    uint8_t  imu_ready;        // 1: IMU calibrada y operativa
    uint8_t  control_status;   // Actuadores encendidos: bit0 ventilador, bit1 motores
    uint16_t checksum;         // Suma de comprobacion
} car_to_base_packet_t;

#pragma pack(pop)

/**
 * @brief Calcula una suma de verificacion de 16 bits para validar paquetes.
 */
static inline uint16_t protocol_checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

#endif // PROTOCOL_DEFS_H
