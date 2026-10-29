#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp_dist;          // 130.0 %/m
    float ki_dist;          // 10.0 %/(m*s)
    float kd_dist;          // 5.0 %*s/m
    float deadband_m;       // 0.005 m (5 mm)

    float kp_theta;         // Ganancia proporcional angulo
    float ki_theta;         // Ganancia integral angulo
    float kd_theta;         // Ganancia derivativa angulo
} controller_gains_t;

typedef struct {
    float u_car_percent;    // Esfuerzo de traccion calculado [-100, 100] %
    float u_fan_percent;    // Esfuerzo de ventilador [10, 90] %
    float error_dist_cm;    // Error de distancia d_est - d_ref [cm]
    float error_theta_deg;  // Error de angulo theta_ref - theta_meas [deg]
} controller_output_t;

/**
 * @brief Inicializa los estados integradores y ganancias de los controladores del carro.
 */
void car_controller_init(void);

/**
 * @brief Reinicia los integradores anti-windup (llamar mientras el actuador este deshabilitado).
 *
 * @param reset_dist Reinicia el integrador del lazo de distancia (motores).
 * @param reset_theta Reinicia el integrador del lazo angular (ventilador).
 */
void car_controller_reset_integrators(bool reset_dist, bool reset_theta);

/**
 * @brief Ejecuta el ciclo de control periódico del carro y del ventilador.
 * 
 * @param d_est_cm Distancia estimada actual por Kalman [cm].
 * @param v_est_ms Velocidad estimada actual por Kalman [m/s].
 * @param theta_meas_deg Angulo actual de la compuerta recibido desde la base [deg].
 * @param d_ref_cm Consigna de distancia deseada [cm].
 * @param theta_ref_deg Consigna de angulo deseada [deg].
 * @param matlab_pwm_feed Señal de control proveniente de MATLAB si mode == 1 [%].
 * @param control_mode 0: Local/Autonomo, 1: Feedback/HIL desde MATLAB.
 * @param dt_s Periodo de muestreo del lazo (ej. 0.01 s = 100 Hz).
 * @return controller_output_t Salidas de control calculadas.
 */
controller_output_t car_controller_step(float d_est_cm,
                                        float v_est_ms,
                                        float theta_meas_deg,
                                        float d_ref_cm,
                                        float theta_ref_deg,
                                        float matlab_pwm_feed,
                                        uint8_t control_mode,
                                        float dt_s);

#endif // CAR_CONTROLLER_H
