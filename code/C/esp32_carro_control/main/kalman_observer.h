#ifndef KALMAN_OBSERVER_H
#define KALMAN_OBSERVER_H

#include <stdint.h>
#include <stdbool.h>

#define CAR_MASS_KG         0.421f   // [kg] Masa total del carro con carga
#define CAR_DAMPING_B       3.500f   // [N*s/m] Amortiguamiento dinamico del chasis
#define CAR_FORCE_MAX_N     3.000f   // [N] Empuje maximo de traccion de los 2 motores
#define CAR_D0_NOMINAL_CM   10.000f  // [cm] Distancia inicial fija al arranque
#define CAR_D_MIN_CM        5.000f   // [cm] Limite mecanico minimo
#define CAR_D_MAX_CM        25.000f  // [cm] Limite mecanico maximo

typedef struct {
    float v_est;        // [m/s] Velocidad lineal estimada
    float b_est;        // [m/s^2] Sesgo dinamico estimado
    float delta_x;      // [m] Desplazamiento acumulado desde d0
    float d_est_cm;     // [cm] Distancia estimada hacia la compuerta: 10cm - delta_x*100
} kalman_state_t;

/**
 * @brief Inicializa el observador de Kalman con la condicion d(0) = 10 cm, v(0) = 0.
 */
void kalman_observer_init(void);

/**
 * @brief Reinicia el estado del observador a la posicion inicial nominal de 10 cm.
 */
void kalman_observer_reset(void);

/**
 * @brief Ejecuta un paso de integracion temporal del observador de estados (100 Hz / dt = 0.01 s).
 * 
 * @param a_meas Aceleracion longitudinal medida por el sensor IMU [m/s^2].
 * @param u_motor Esfuerzo de control actual de los motores de traccion [-100.0 a +100.0 %].
 * @param dt_s Intervalo de integracion (0.01 s).
 * @return kalman_state_t Estructura con el estado actualizado (distancia, velocidad, sesgo).
 */
kalman_state_t kalman_observer_update(float a_meas, float u_motor, float dt_s);

/**
 * @brief Obtiene el estado actual del observador.
 */
kalman_state_t kalman_observer_get_state(void);

#endif // KALMAN_OBSERVER_H
