// Pruebas del controlador y del observador del carro, sin hardware.
// En PC con gcc, desde code/C:
//   gcc -I esp32_carro_control/main test/test_control.c esp32_carro_control/main/car_controller.c esp32_carro_control/main/lqr_simo.c esp32_carro_control/main/kalman_observer.c -lm -o test_control && ./test_control
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "car_controller.h"
#include "kalman_observer.h"

static const float DT = 0.01f;

// Lazo cerrado: LQR+I del carro contra la compuerta no lineal de init_parametros.m (carro quieto en d).
// El angulo llega con un ciclo de retardo (base -> ESP-NOW -> carro) y cuantizado como el ADC.
typedef struct { float t_sub; float sobrepico; float final; float pwm_min; float pwm_max; } resultado_t;

static resultado_t lazo_cerrado(float theta_ref_deg, float d_cm, float t_fin) {
    const float J = 2e-4f, B = 1e-3f, TF = 1e-4f, MGL = 0.014715f, KM = 3e-4f, TFAN = 0.15f;
    const float Q = 180.0f / 4095.0f; // grados por cuenta del ADC
    float th = 0.0f, om = 0.0f, vf = 0.0f, th_meas = 0.0f;
    resultado_t r = {-1.0f, 0.0f, 0.0f, 100.0f, 0.0f};
    car_controller_init(0.0f);
    for (int k = 0; k < (int)(t_fin / DT); k++) {
        controller_output_t o = car_controller_step(d_cm, 0.0f, th_meas, theta_ref_deg, true, false, DT);
        th_meas = roundf(th * 57.2957795f / Q) * Q; // se usa en el ciclo siguiente
        for (int s = 0; s < 10; s++) { // planta integrada a 1 kHz
            float tau = KM * vf * (0.10f / (d_cm / 100.0f)) * (0.10f / (d_cm / 100.0f));
            float acc = (tau - MGL * sinf(th) - B * om - TF * tanhf(50.0f * om)) / J;
            om += 0.001f * acc;
            th += 0.001f * om;
            vf += 0.001f * (o.u_fan_percent - vf) / TFAN;
        }
        float th_deg = th * 57.2957795f, t = (k + 1) * DT;
        if (r.t_sub < 0 && th_deg >= 0.9f * theta_ref_deg) r.t_sub = t;
        if (th_deg - theta_ref_deg > r.sobrepico) r.sobrepico = th_deg - theta_ref_deg;
        if (o.u_fan_percent < r.pwm_min) r.pwm_min = o.u_fan_percent;
        if (o.u_fan_percent > r.pwm_max) r.pwm_max = o.u_fan_percent;
        r.final = th_deg;
    }
    return r;
}

int main(void) {
    // Signo del lazo de distancia: con el ventilador apagado d_ref = d_est, asi que no hay esfuerzo
    car_controller_init(0.0f);
    controller_output_t o = car_controller_step(15.0f, 0.0f, 0.0f, 35.0f, false, true, DT);
    assert(o.u_fan_percent == 0.0f && fabsf(o.error_dist_cm) < 1e-4f && o.u_car_percent == 0.0f);

    // Motores apagados: sin esfuerzo aunque haya error
    car_controller_init(0.0f);
    o = car_controller_step(15.0f, 0.0f, 0.0f, 35.0f, true, false, DT);
    assert(o.u_car_percent == 0.0f);

    // Ventilador encendido: PWM siempre en [10, 90] % y NaN en la consigna no se propaga
    car_controller_init(0.0f);
    o = car_controller_step(10.0f, 0.0f, 0.0f, NAN, true, false, DT);
    assert(o.u_fan_percent >= 10.0f && o.u_fan_percent <= 90.0f);

    // LQR+I en lazo cerrado. El .slx (controlador continuo, sin retardo) da sobrepico 0.9 deg y PWM max 65 %;
    // a 100 Hz y con 1 ciclo de retardo del angulo la simulacion equivalente en MATLAB da t90 0.31 s,
    // sobrepico 2.5 deg, error final 0.004 deg y PWM max 73 %.
    resultado_t r = lazo_cerrado(35.0f, 10.0f, 4.0f);
    printf("35 deg / 10 cm: t90 %.2f s, sobrepico %.2f deg, final %.3f deg, PWM %.1f-%.1f %%\n",
           r.t_sub, r.sobrepico, r.final, r.pwm_min, r.pwm_max);
    assert(r.t_sub > 0.0f && r.t_sub < 0.8f);
    assert(r.sobrepico < 3.0f);
    assert(fabsf(r.final - 35.0f) < 0.2f);
    assert(r.pwm_min >= 10.0f && r.pwm_max <= 90.0f);

    // Otro punto de la rejilla de ganancias (verifica la interpolacion de la tabla K)
    r = lazo_cerrado(20.0f, 15.0f, 4.0f);
    printf("20 deg / 15 cm: t90 %.2f s, sobrepico %.2f deg, final %.3f deg, PWM %.1f-%.1f %%\n",
           r.t_sub, r.sobrepico, r.final, r.pwm_min, r.pwm_max);
    assert(r.t_sub > 0.0f && fabsf(r.final - 20.0f) < 0.2f && r.sobrepico < 3.0f);

    // Observador en reposo (u = 0, a = 0): la distancia no deriva de 10 cm
    kalman_observer_init();
    kalman_state_t k = kalman_observer_get_state();
    for (int i = 0; i < 1000; i++) k = kalman_observer_update(0.0f, 0.0f, DT);
    assert(fabsf(k.d_est_cm - CAR_D0_NOMINAL_CM) < 1e-3f && k.v_est == 0.0f);

    // Empuje positivo con aceleracion medida positiva: el carro avanza y d disminuye
    kalman_observer_init();
    for (int i = 0; i < 10; i++) k = kalman_observer_update(1.0f, 50.0f, DT); // corto: sin llegar al tope de 5 cm
    assert(k.v_est > 0.0f && k.d_est_cm < CAR_D0_NOMINAL_CM);

    puts("OK: test_control");
    return 0;
}

#ifdef ESP_PLATFORM
// Sin gcc para PC: idf.py -C code/C/test qemu (o flash en un ESP32) ejecuta la misma prueba
void app_main(void) { main(); }
#endif
