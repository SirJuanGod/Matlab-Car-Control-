#include "car_controller.h"
#include "lqr_simo.h"
#include <math.h>

#define DEG2RAD 0.0174532925f

// PID de distancia del carro (init_parametros.m: car_Kp, car_Ki, car_Kd, car_deadband)
static const float KP_DIST = 130.0f;      // %/m
static const float KI_DIST = 10.0f;       // %/(m*s)
static const float KD_DIST = 5.0f;        // %*s/m
static const float DEADBAND_M = 0.005f;   // 5 mm
static const float FAN_PWM_MIN = 10.0f;   // %, pwm_min (sin incluir drivers: se prueba en PC)
static const float INT_DIST_LIM = 30.0f / 10.0f; // integral limitada a +-30 % (30/Ki), como el .slx

static float s_int_error_dist = 0.0f;

void car_controller_init(float theta_meas_deg) {
    s_int_error_dist = 0.0f;
    lqr_simo_reset(theta_meas_deg * DEG2RAD);
}

controller_output_t car_controller_step(float d_est_cm,
                                        float v_est_ms,
                                        float theta_meas_deg,
                                        float theta_ref_deg,
                                        bool fan_on,
                                        bool motors_on,
                                        float dt_s)
{
    controller_output_t out;

    // 1. Lazo angular: LQR+I SIMO -> PWM del ventilador y consigna de distancia
    float d_ref_m = d_est_cm / 100.0f; // ventilador apagado: el carro se queda donde esta
    out.u_fan_percent = 0.0f;
    if (fan_on) {
        lqr_simo_out_t lqr = lqr_simo_step(theta_ref_deg * DEG2RAD, theta_meas_deg * DEG2RAD,
                                           d_est_cm / 100.0f, dt_s);
        out.u_fan_percent = lqr.u_pwm;
        d_ref_m = lqr.d_ref_m;
    } else {
        lqr_simo_reset(theta_meas_deg * DEG2RAD); // sin acumular error: arranca suave al encender
    }
    out.d_ref_cm = d_ref_m * 100.0f;

    // 2. Lazo de posicion del carro: e_a = d_est - d_ref (> 0 pide avanzar), derivada sobre v
    float e_dist_m = (d_est_cm - out.d_ref_cm) / 100.0f;
    out.error_dist_cm = d_est_cm - out.d_ref_cm;
    if (fabsf(e_dist_m) < DEADBAND_M) e_dist_m = 0.0f;

    if (motors_on) {
        s_int_error_dist += e_dist_m * dt_s;
        if (s_int_error_dist > INT_DIST_LIM) s_int_error_dist = INT_DIST_LIM;
        if (s_int_error_dist < -INT_DIST_LIM) s_int_error_dist = -INT_DIST_LIM;
    } else {
        s_int_error_dist = 0.0f;
    }

    float u_car = KP_DIST * e_dist_m + KI_DIST * s_int_error_dist - KD_DIST * v_est_ms;
    if (u_car > 100.0f) u_car = 100.0f;
    if (u_car < -100.0f) u_car = -100.0f;
    out.u_car_percent = motors_on ? u_car : 0.0f;

    // Proteccion contra NaN / Inf (p. ej. una consigna corrupta): actuadores al minimo seguro
    if (!isfinite(out.u_fan_percent)) {
        out.u_fan_percent = fan_on ? FAN_PWM_MIN : 0.0f;
        lqr_simo_reset(theta_meas_deg * DEG2RAD);
    }
    if (!isfinite(out.u_car_percent)) {
        out.u_car_percent = 0.0f;
        s_int_error_dist = 0.0f;
    }
    return out;
}
