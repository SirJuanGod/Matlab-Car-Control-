#include "car_controller.h"
#include <math.h>

#define PI_CONST 3.14159265358979323846f

// Parametros fisicos identicos a init_parametros.m
static const float M_G = 0.015f;        // [kg]
static const float G_ACCEL = 9.81f;     // [m/s^2]
static const float L_C = 0.10f;         // [m]
static const float K_M = 0.00030f;      // [N*m/%]
static const float D0_CM = 10.0f;       // [cm]

static controller_gains_t s_gains = {
    .kp_dist = 130.0f,     // %/m
    .ki_dist = 10.0f,      // %/(m*s)
    .kd_dist = 5.0f,       // %*s/m
    .deadband_m = 0.005f,  // 5 mm

    .kp_theta = 1.25f,     // Ganancia proporcional de la compuerta
    .ki_theta = 0.40f,     // Ganancia integral de la compuerta
    .kd_theta = 0.08f      // Derivada sobre el angulo
};

static float s_int_error_dist = 0.0f;
static float s_int_error_theta = 0.0f;
static float s_last_theta_meas = 0.0f;

void car_controller_init(void) {
    car_controller_reset_integrators();
}

void car_controller_reset_integrators(void) {
    s_int_error_dist = 0.0f;
    s_int_error_theta = 0.0f;
    s_last_theta_meas = 0.0f;
}

controller_output_t car_controller_step(float d_est_cm,
                                        float v_est_ms,
                                        float theta_meas_deg,
                                        float d_ref_cm,
                                        float theta_ref_deg,
                                        float matlab_pwm_feed,
                                        uint8_t control_mode,
                                        float dt_s)
{
    controller_output_t out;

    // =========================================================================
    // 1. Lazo de Posicion del Carro (PID con Odometria Kalman)
    //    e_a = d_est - d_ref (en metros)
    //    u_car = Kp*e_a + Ki*int(e_a) - Kd*v_adv
    // =========================================================================
    float e_dist_m = (d_est_cm - d_ref_cm) / 100.0f;
    out.error_dist_cm = d_est_cm - d_ref_cm;

    // Aplicacion de banda muerta (+-5 mm)
    if (fabsf(e_dist_m) < s_gains.deadband_m) {
        e_dist_m = 0.0f;
    }

    // Integrador con proteccion anti-windup
    s_int_error_dist += e_dist_m * dt_s;
    if (s_int_error_dist > 2.0f) s_int_error_dist = 2.0f;
    if (s_int_error_dist < -2.0f) s_int_error_dist = -2.0f;

    // Calculo del esfuerzo del carro
    float u_car = (s_gains.kp_dist * e_dist_m) +
                  (s_gains.ki_dist * s_int_error_dist) -
                  (s_gains.kd_dist * v_est_ms);

    // Saturacion de motores [-100%, +100%]
    if (u_car > 100.0f) u_car = 100.0f;
    if (u_car < -100.0f) u_car = -100.0f;
    out.u_car_percent = u_car;

    // =========================================================================
    // 2. Lazo de Control Angular / Ventilador San Ace 40
    // =========================================================================
    float u_fan = 0.0f;
    out.error_theta_deg = theta_ref_deg - theta_meas_deg;

    if (control_mode == 1) {
        // MODO HIL / Feedback de simulacion desde MATLAB:
        u_fan = matlab_pwm_feed;
    } else {
        // MODO AUTONOMO (SIMO con compensacion aerodinamica cuadrática inversa)
        // Ganancia aerodinamica local: kd_j = k_m * (d0 / d_est)^2
        float d_ratio = (d_est_cm > 1.0f) ? (D0_CM / d_est_cm) : 1.0f;
        float kd_local = K_M * (d_ratio * d_ratio);
        if (kd_local < 1e-6f) kd_local = 1e-6f;

        // PWM de equilibrio estatico gravitacional: u_eq = (m*g*lc*sin(th)) / kd_local
        float th_ref_rad = theta_ref_deg * (PI_CONST / 180.0f);
        float tau_gravity = M_G * G_ACCEL * L_C * sinf(th_ref_rad);
        float u_eq = tau_gravity / kd_local;

        // PID sobre el angulo
        float e_th = out.error_theta_deg;
        s_int_error_theta += e_th * dt_s;
        if (s_int_error_theta > 50.0f) s_int_error_theta = 50.0f;
        if (s_int_error_theta < -50.0f) s_int_error_theta = -50.0f;

        float d_theta_dt = (dt_s > 0.0f) ? ((theta_meas_deg - s_last_theta_meas) / dt_s) : 0.0f;
        s_last_theta_meas = theta_meas_deg;

        u_fan = u_eq + (s_gains.kp_theta * e_th) + (s_gains.ki_theta * s_int_error_theta) - (s_gains.kd_theta * d_theta_dt);
    }

    // Limites de operacion del ventilador: [10%, 90%] segun especificaciones_ventilador.md
    if (u_fan > 90.0f) u_fan = 90.0f;
    if (u_fan < 10.0f) u_fan = 10.0f;

    // Proteccion contra NaN / Inf en punto flotante para hardware embebido
    if (isnan(u_fan) || isinf(u_fan)) {
        u_fan = 10.0f;
    }
    if (isnan(out.u_car_percent) || isinf(out.u_car_percent)) {
        out.u_car_percent = 0.0f;
    }

    out.u_fan_percent = u_fan;
    return out;
}
