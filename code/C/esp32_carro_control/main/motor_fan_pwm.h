#ifndef MOTOR_FAN_PWM_H
#define MOTOR_FAN_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Pines de Actuacion Ventilador (L298N 24V)
#define FAN_PWM_PIN            18
#define FAN_DIR_PIN            19

// Pines de Motores de Traccion (L298N 7.4V)
#define MOTOR_LEFT_PWM_PIN     25
#define MOTOR_LEFT_IN1_PIN     27
#define MOTOR_LEFT_IN2_PIN     14

// Pines seguros sin conflictos de strapping al arranque
#define MOTOR_RIGHT_PWM_PIN    26
#define MOTOR_RIGHT_IN3_PIN    32
#define MOTOR_RIGHT_IN4_PIN    33

// Limites de saturacion segun especificaciones_ventilador.md
#define FAN_PWM_MIN_PERCENT    10.0f
#define FAN_PWM_MAX_PERCENT    90.0f

/**
 * @brief Configura el periferico LEDC de hardware para control PWM a 5 kHz (10 bits) y pines GPIO.
 * 
 * @return esp_err_t ESP_OK si la configuracion fue exitosa.
 */
esp_err_t motor_fan_pwm_init(void);

/**
 * @brief Aplica el ciclo de trabajo al ventilador San Ace 40.
 * 
 * @param percent Ciclo de trabajo [10.0% a 90.0%]. Si percent <= 0.0f o enable es falso, se apaga.
 * @param enabled Habilita o deshabilita el accionamiento.
 */
void fan_set_pwm_percent(float percent, bool enabled);

/**
 * @brief Comanda ambos motores de traccion diferencial.
 * 
 * @param u_percent Esfuerzo de traccion [-100.0% a +100.0%].
 *                  > 0: Avance hacia la compuerta.
 *                  < 0: Retroceso alejandose de la compuerta.
 *                  0: Parada suave.
 * @param enabled Habilita o frena los motores.
 */
void car_set_motors(float u_percent, bool enabled);

/**
 * @brief Parada de emergencia segura de todos los actuadores.
 */
void actuators_emergency_stop(void);

#endif // MOTOR_FAN_PWM_H
