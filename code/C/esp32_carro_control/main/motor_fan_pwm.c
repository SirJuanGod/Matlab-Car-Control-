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

void car_set_motors(float u_percent, bool enabled) {
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

    if (u_percent > 100.0f) u_percent = 100.0f;
    if (u_percent < -100.0f) u_percent = -100.0f;

    // Compensacion de zona muerta por friccion estatica mecanica (stiction de motorreductores 1:48)
    // Los motores TGP01D-A130 requieren un ciclo minimo ~20% para vencer el rozamiento
    float mag = fabsf(u_percent);
    float duty_percent = 20.0f + (mag * 0.80f); // Escala lineal de 20% a 100%
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t duty = (uint32_t)((duty_percent / 100.0f) * (float)PWM_MAX_DUTY_COUNT);

    if (u_percent > 0.0f) {
        // Avance hacia la compuerta (reduce d)
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 1);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 1);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);
    } else {
        // Retroceso alejandose de la compuerta (aumenta d)
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 0);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 1);
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 1);
    }

    ledc_set_duty(PWM_MODE, MOTOR_LEFT_CHANNEL, duty);
    ledc_set_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, MOTOR_LEFT_CHANNEL);
    ledc_update_duty(PWM_MODE, MOTOR_RIGHT_CHANNEL);
}

void actuators_emergency_stop(void) {
    fan_set_pwm_percent(0.0f, false);
    car_set_motors(0.0f, false);
}
