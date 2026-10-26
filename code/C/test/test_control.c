// Pruebas en PC (sin ESP-IDF) del controlador y del observador del carro.
// Compilar y ejecutar desde code/C:
//   gcc -I esp32_carro_control/main test/test_control.c esp32_carro_control/main/car_controller.c esp32_carro_control/main/kalman_observer.c -lm -o test_control && ./test_control
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "car_controller.h"
#include "kalman_observer.h"

static const float DT = 0.01f;

static controller_output_t step(float d_est, float d_ref) {
    return car_controller_step(d_est, 0.0f, 35.0f, d_ref, 35.0f, 0.0f, 0, DT);
}

int main(void) {
    // Signo del lazo de distancia: lejos de la consigna -> avanzar (u > 0), cerca -> retroceder (u < 0)
    car_controller_init();
    assert(step(15.0f, 10.0f).u_car_percent > 0.0f);
    car_controller_init();
    assert(step(6.0f, 10.0f).u_car_percent < 0.0f);

    // Banda muerta de 5 mm: sin esfuerzo
    car_controller_init();
    assert(step(10.3f, 10.0f).u_car_percent == 0.0f);

    // Ventilador siempre en [10, 90] % y NaN en la consigna no se propaga a la salida
    car_controller_init();
    controller_output_t o = car_controller_step(10.0f, 0.0f, 0.0f, 10.0f, NAN, 0.0f, 0, DT);
    assert(o.u_fan_percent >= 10.0f && o.u_fan_percent <= 90.0f);

    // El reset del integrador de distancia elimina el esfuerzo acumulado
    car_controller_init();
    float u_first = step(15.0f, 10.0f).u_car_percent;
    for (int i = 0; i < 500; i++) step(15.0f, 10.0f);
    assert(step(15.0f, 10.0f).u_car_percent > u_first);
    car_controller_reset_integrators(true, false);
    assert(fabsf(step(15.0f, 10.0f).u_car_percent - u_first) < 0.5f);

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
