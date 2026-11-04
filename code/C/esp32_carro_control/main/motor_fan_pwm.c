#include "motor_fan_pwm.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "ACTUATORS_PWM";

#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ         5000
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY_COUNT  1023 // 2^10 - 1

#define FAN_CHANNEL         LEDC_CHANNEL_0
#define MOTOR_LEFT_CHANNEL  LEDC_CHANNEL_1
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_2

esp_err_t motor_fan_pwm_init(void) {
    // 1. Configurar GPIOs de direccion
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FAN_DIR_PIN) |
                        (1ULL << MOTOR_LEFT_IN1_PIN) | (1ULL << MOTOR_LEFT_IN2_PIN) |
                        (1ULL << MOTOR_RIGHT_IN3_PIN) | (1ULL << MOTOR_RIGHT_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Estados seguros iniciales (todo apagado)
    gpio_set_level(FAN_DIR_PIN, 0);
    gpio_set_level(MOTOR_LEFT_IN1_PIN, 0);
    gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_IN3_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);

    // 2. Configurar Timer LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 3. Configurar Canales LEDC
    ledc_channel_config_t ch_fan = {
        .gpio_num = FAN_PWM_PIN,
        .speed_mode = PWM_MODE,
        .channel = FAN_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_fan));

    ledc_channel_config_t ch_m_left = {
        .gpio_num = MOTOR_LEFT_PWM_PIN,
        .speed_mode = PWM_MODE,
        .channel = MOTOR_LEFT_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_m_left));

    ledc_channel_config_t ch_m_right = {
        .gpio_num = MOTOR_RIGHT_PWM_PIN,
        .speed_mode = PWM_MODE,
        .channel = MOTOR_RIGHT_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_m_right));

    ESP_LOGI(TAG, "LEDC PWM Inicializado (5 kHz, 10 bits: Ventilador GPIO%d, Motores GPIO%d/%d).",
             FAN_PWM_PIN, MOTOR_LEFT_PWM_PIN, MOTOR_RIGHT_PWM_PIN);
    return ESP_OK;
}

void fan_set_pwm_percent(float percent, bool enabled) {
    if (!enabled || percent < 1.0f) {
        ledc_set_duty(PWM_MODE, FAN_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, FAN_CHANNEL);
        gpio_set_level(FAN_DIR_PIN, 0); // Apagar excitacion del puente
        return;
    }

    // Saturacion estricta [10%, 90%] segun especificaciones_ventilador.md
    if (percent < FAN_PWM_MIN_PERCENT) percent = FAN_PWM_MIN_PERCENT;
    if (percent > FAN_PWM_MAX_PERCENT) percent = FAN_PWM_MAX_PERCENT;

    uint32_t duty = (uint32_t)((percent / 100.0f) * (float)PWM_MAX_DUTY_COUNT);
    // Para el L298N: ENA recibe PWM y DIR (IN1) debe estar en ALTO (1) con IN2 en GND (0)
    gpio_set_level(FAN_DIR_PIN, 1);
    ledc_set_duty(PWM_MODE, FAN_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, FAN_CHANNEL);
}

float car_motor_effective_percent(float u_percent) {
    if (fabsf(u_percent) < 1.5f) return 0.0f;
    float mag = fabsf(u_percent);
    if (mag > 100.0f) mag = 100.0f;
    float duty = MOTOR_PWM_MIN_PERCENT + (mag / 100.0f) * (MOTOR_PWM_MAX_PERCENT - MOTOR_PWM_MIN_PERCENT);
    if (duty > 100.0f) duty = 100.0f;
    return (u_percent > 0.0f) ? duty : -duty;
}

void car_set_motors(float u_percent, bool enabled) {
    car_set_motors_steer(u_percent, 0.0f, enabled);
}

void car_set_motors_steer(float u_percent, float steer_percent, bool enabled) {
    if (!enabled || fabsf(u_percent) < 1.5f) {
        // Parada / frenado pasivo (todos los pines de puente a 0)
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 0);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);
        ledc_set_duty(PWM_MODE, MOTOR_LEFT_CHANNEL, 0);
        ledc_set_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, MOTOR_LEFT_CHANNEL);
        ledc_update_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL);
        return;
    }

    // Compensacion de zona muerta por friccion estatica mecanica (stiction de motorreductores 1:48)
    // En pruebas el carro no avanzaba con ciclos bajos: se usa un minimo de MOTOR_PWM_MIN_PERCENT
    float duty_percent = fabsf(car_motor_effective_percent(u_percent)); // Escala lineal de MIN a MAX

    // Correccion de rumbo: steer > 0 acelera la rueda derecha y frena la izquierda
    // (en duty absoluto). Se limita para no bajar de la friccion estatica ni pasar de 100 %.
    if (steer_percent > MOTOR_STEER_MAX_PERCENT) steer_percent = MOTOR_STEER_MAX_PERCENT;
    if (steer_percent < -MOTOR_STEER_MAX_PERCENT) steer_percent = -MOTOR_STEER_MAX_PERCENT;
    float duty_left = duty_percent - steer_percent;
    float duty_right = duty_percent + steer_percent;
    // Si una rueda se pasaria de 100 %, se baja la otra lo que falta (mantiene la diferencia)
    if (duty_right > 100.0f) { duty_left -= duty_right - 100.0f; duty_right = 100.0f; }
    if (duty_left > 100.0f)  { duty_right -= duty_left - 100.0f; duty_left = 100.0f; }
    if (duty_left < MOTOR_STEER_MIN_DUTY)  duty_left = MOTOR_STEER_MIN_DUTY;
    if (duty_right < MOTOR_STEER_MIN_DUTY) duty_right = MOTOR_STEER_MIN_DUTY;

    uint32_t duty_l = (uint32_t)((duty_left / 100.0f) * (float)PWM_MAX_DUTY_COUNT);
    uint32_t duty_r = (uint32_t)((duty_right / 100.0f) * (float)PWM_MAX_DUTY_COUNT);

    // u > 0: avance hacia la compuerta (reduce d); u < 0: retroceso (aumenta d).
    // Con el cableado actual IN1=1/IN3=1 mueve el carro hacia atras (prueba de direccion),
    // por eso MOTOR_DIR_INVERT invierte el sentido de ambos puentes.
    bool in_a = (u_percent > 0.0f) != (MOTOR_DIR_INVERT != 0);
    gpio_set_level(MOTOR_LEFT_IN1_PIN, in_a);
    gpio_set_level(MOTOR_LEFT_IN2_PIN, !in_a);
    gpio_set_level(MOTOR_RIGHT_IN3_PIN, in_a);
    gpio_set_level(MOTOR_RIGHT_IN4_PIN, !in_a);

    ledc_set_duty(PWM_MODE, MOTOR_LEFT_CHANNEL, duty_l);
    ledc_set_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL, duty_r);
    ledc_update_duty(PWM_MODE, MOTOR_LEFT_CHANNEL);
    ledc_update_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL);
}

void actuators_emergency_stop(void) {
    fan_set_pwm_percent(0.0f, false);
    car_set_motors(0.0f, false);
}
