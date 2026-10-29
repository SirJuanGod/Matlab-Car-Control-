#ifndef LQR_SIMO_H
#define LQR_SIMO_H

/**
 * Controlador LQR+I SIMO del angulo de la compuerta, igual al bloque Controlador_SIMO de
 * modelo_miso_compuerta.slx: prefiltro + gobernador de la referencia, estimadores de omega y
 * v_fan, ganancias programadas K(theta_ref, d_nom), anti-windup y supervisor de distancia.
 * Ganancias y parametros en lqr_tablas.h (generado por code/MATLAB/exportar_lqr_c.m).
 */

typedef struct {
    float u_pwm;      // PWM del ventilador saturado [10, 90] %
    float d_ref_m;    // consigna de distancia para el lazo del carro [m]
    float p_raw;      // PWM pedido antes de saturar [%]
    float theta_ref;  // referencia prefiltrada que sigue el controlador [rad]
} lqr_simo_out_t;

/**
 * @brief Reinicia los estados (integral, d_nom, estimadores, referencia) en reposo.
 *        Llamar al arrancar y mientras el ventilador este deshabilitado.
 * @param theta_meas Angulo medido actual [rad].
 */
void lqr_simo_reset(float theta_meas);

/**
 * @brief Un paso del controlador (Euler, periodo dt).
 * @param theta_cmd  Consigna de angulo pedida por el usuario [rad].
 * @param theta_meas Angulo medido por el potenciometro [rad].
 * @param d_est      Distancia estimada por la IMU [m].
 * @param dt         Periodo de muestreo [s].
 */
lqr_simo_out_t lqr_simo_step(float theta_cmd, float theta_meas, float d_est, float dt);

#endif // LQR_SIMO_H
