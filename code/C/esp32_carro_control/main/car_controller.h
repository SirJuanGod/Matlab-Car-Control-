#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float u_car_percent;    // Esfuerzo de traccion calculado [-100, 100] %
    float u_fan_percent;    // Esfuerzo de ventilador [10, 90] % (0 con el ventilador apagado)
    float d_ref_cm;         // Consigna de distancia que fija el LQR SIMO [cm]
    float error_dist_cm;    // Error de distancia d_est - d_ref [cm]
} controller_output_t;

/**
 * @brief Deja el controlador en reposo: integradores a cero y referencia en el angulo actual.
 */
void car_controller_init(float theta_meas_deg);

/**
 * @brief Un ciclo de control: LQR+I SIMO del angulo (PWM del ventilador y d_ref) y PID de distancia del carro.
 *
 * Mientras un actuador esta apagado su controlador se mantiene en reposo (sin acumular error):
 * con el ventilador apagado el LQR se reinicia y el carro se queda donde esta.
 *
 * @param d_est_cm Distancia estimada por Kalman [cm].
 * @param v_est_ms Velocidad estimada por Kalman [m/s].
 * @param theta_meas_deg Angulo de la compuerta recibido de la base [deg].
 * @param theta_ref_deg Consigna de angulo [deg].
 * @param fan_on Ventilador habilitado.
 * @param motors_on Motores habilitados.
 * @param dt_s Periodo de muestreo [s].
 */
controller_output_t car_controller_step(float d_est_cm,
                                        float v_est_ms,
                                        float theta_meas_deg,
                                        float theta_ref_deg,
                                        bool fan_on,
                                        bool motors_on,
                                        float dt_s);

#endif // CAR_CONTROLLER_H
