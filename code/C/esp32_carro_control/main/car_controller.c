#include "car_controller.h"
#include "lqr_simo.h"
#include <math.h>

#define DEG2RAD 0.0174532925f

// PID de distancia del carro (init_parametros.m: car_Kp, car_Ki, car_Kd, car_deadband)
static const float KP_DIST = 130.0f;      // %/m
static const float KI_DIST = 10.0f;       // %/(m*s)
static const float KD_DIST = 5.0f;        // %*s/m
static const float DEADBAND_M = 0.010f;   // 10 mm: el ruido/deriva de la IMU no debe mover el carro
// Los motores solo vencen la friccion con >= 80 % de PWM, asi que cualquier u_car pequeno produce un
// arranque brusco. Histeresis: se encienden con |u| >= U_CAR_ON y se apagan con |u| < U_CAR_OFF.
static const float U_CAR_ON = 8.0f;       // %
static const float U_CAR_OFF = 4.0f;      // %
static bool s_car_moving = false;
static const float FAN_PWM_MIN = 10.0f;   // %, pwm_min (sin incluir drivers: se prueba en PC)
static const float INT_DIST_LIM = 30.0f / 10.0f; // integral limitada a +-30 % (30/Ki), como el .slx

static float s_int_error_dist = 0.0f;

void car_controller_init(float theta_meas_deg) {
    s_int_error_dist = 0.0f;
    s_car_moving = false;
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
    bool en_banda = fabsf(e_dist_m) < DEADBAND_M;
    if (en_banda) e_dist_m = 0.0f;

    if (motors_on && !en_banda) {
        // Dentro de la banda muerta la integral se congela: si siguiera acumulando
        // terminaria disparando un avance aunque el carro ya este en su sitio
        s_int_error_dist += e_dist_m * dt_s;
        if (s_int_error_dist > INT_DIST_LIM) s_int_error_dist = INT_DIST_LIM;
        if (s_int_error_dist < -INT_DIST_LIM) s_int_error_dist = -INT_DIST_LIM;
    } else if (!motors_on) {
        s_int_error_dist = 0.0f;
    }

    float u_car = KP_DIST * e_dist_m + KI_DIST * s_int_error_dist - KD_DIST * v_est_ms;
    if (u_car > 100.0f) u_car = 100.0f;
    if (u_car < -100.0f) u_car = -100.0f;

    // Histeresis de arranque/parada de los motores
    float mag = fabsf(u_car);
    if (!motors_on || en_banda) {
        s_car_moving = false;
    } else if (s_car_moving) {
        if (mag < U_CAR_OFF) s_car_moving = false;
    } else if (mag >= U_CAR_ON) {
        s_car_moving = true;
    }
    out.u_car_percent = s_car_moving ? u_car : 0.0f;

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
