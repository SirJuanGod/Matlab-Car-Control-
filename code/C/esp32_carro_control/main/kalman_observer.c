#include "kalman_observer.h"
#include <math.h>

static kalman_state_t s_state = {
    .v_est = 0.0f,
    .b_est = 0.0f,
    .delta_x = 0.0f,
    .d_est_cm = CAR_D0_NOMINAL_CM
};

// Ganancias del observador LQE calculadas para el modelo lineal del carro
static const float L1_GAIN = 0.852f; // Correccion sobre velocidad
static const float L2_GAIN = 0.148f; // Correccion sobre sesgo de aceleracion

void kalman_observer_init(void) {
    kalman_observer_reset();
}

void kalman_observer_reset(void) {
    s_state.v_est = 0.0f;
    s_state.b_est = 0.0f;
    s_state.delta_x = 0.0f;
    s_state.d_est_cm = CAR_D0_NOMINAL_CM;
}

kalman_state_t kalman_observer_update(float a_meas, float u_motor, float dt_s) {
    // Verificacion de reposo fisico (ZUPT: Zero Velocity Update)
    // En el montaje fisico, si los motores no estan excitados (|u| < 1.5%) y no hay un empuje externo
    // fuerte (|a_meas - bias| < 0.18 m/s^2), el carro permanece inmovil por friccion mecanica de las cajas 1:48.
    bool is_physically_stopped = (fabsf(u_motor) < 1.5f) && (fabsf(a_meas - s_state.b_est) < 0.18f);

    if (is_physically_stopped) {
        // Amortiguar velocidad a cero para eliminar la deriva de doble integracion
        s_state.v_est *= 0.80f;
        if (fabsf(s_state.v_est) < 0.005f) {
            s_state.v_est = 0.0f;
        }
        // El sesgo sigue adaptandose suavemente para absorber desviaciones termicas o ligera inclinacion
        s_state.b_est += 0.015f * (a_meas - s_state.b_est);
    } else {
        // 1. Fuerza de traccion calculada a partir del esfuerzo porcentual de los motores [-100, 100] %
        // Positivo: avanza hacia la compuerta (+ax, reduce d)
        // Negativo: retrocede alejandose de la compuerta (-ax, aumenta d)
        float f_trac = (u_motor / 100.0f) * CAR_FORCE_MAX_N;

        // 2. Aceleracion esperada segun el modelo fisico: a_hat = (F - b*v)/m + b_est
        float a_model = (f_trac - (CAR_DAMPING_B * s_state.v_est)) / CAR_MASS_KG;
        float a_hat = a_model + s_state.b_est;

        // 3. Error de innovacion entre la medicion de la IMU y el modelo estimado
        float innov = a_meas - a_hat;

        // 4. Derivadas de los estados segun el observador LQE
        float v_dot = a_model + (L1_GAIN * innov);
        float b_dot = (L2_GAIN * innov);

        // 5. Integracion de Euler de los estados
        s_state.v_est += v_dot * dt_s;
        s_state.b_est += b_dot * dt_s;
        s_state.delta_x += s_state.v_est * dt_s;
    }

    // 6. Calculo de la distancia estimada: d(t) = d0 - Delta_x
    s_state.d_est_cm = CAR_D0_NOMINAL_CM - (s_state.delta_x * 100.0f);

    // 7. Limitadores fisicos de carrera mecanica [5 cm, 25 cm]
    if (s_state.d_est_cm < CAR_D_MIN_CM) {
        s_state.d_est_cm = CAR_D_MIN_CM;
        s_state.delta_x = (CAR_D0_NOMINAL_CM - CAR_D_MIN_CM) / 100.0f;
        if (s_state.v_est > 0.0f) s_state.v_est = 0.0f; // Detener avance al tocar tope
    } else if (s_state.d_est_cm > CAR_D_MAX_CM) {
        s_state.d_est_cm = CAR_D_MAX_CM;
        s_state.delta_x = (CAR_D0_NOMINAL_CM - CAR_D_MAX_CM) / 100.0f;
        if (s_state.v_est < 0.0f) s_state.v_est = 0.0f; // Detener retroceso al tocar tope
    }

    return s_state;
}

kalman_state_t kalman_observer_get_state(void) {
    return s_state;
}
