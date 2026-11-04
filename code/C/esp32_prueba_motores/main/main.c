// Pruebas de banco de los motores de traccion. Pines iguales a esp32_carro_control/main/motor_fan_pwm.h.
//  TEST_MODE 0: barrido escalonado del PWM (avance) para ver desde que ciclo arrancan.
//  TEST_MODE 1: prueba de direccion. Un pulso "AVANCE" tal como lo manda el firmware del carro
//               (u > 0: IN1=1, IN3=1) con el LED D2 encendido, y un pulso "RETROCESO" con el LED
//               apagado. La IMU integra la velocidad de cada pulso para saber hacia donde se movio.
// El ventilador queda apagado.
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "mpu6050.h"

#define TEST_MODE           1

#define DIR_WAIT_MS         5000  // tiempo para colocar el carro antes de calibrar
#define DIR_PULSE_MS        600   // duracion de cada pulso de motor
#define DIR_PULSE_PERCENT   85    // PWM de cada pulso
#define DIR_SETTLE_MS       1500  // pausa entre pulsos (se sigue integrando 400 ms tras el pulso)
#define LED_PIN             2

#define SWEEP_START_PERCENT 20    // Primer escalon
#define SWEEP_END_PERCENT   100   // Ultimo escalon
#define SWEEP_STEP_PERCENT  10    // Incremento entre escalones
#define SWEEP_STEP_MS       3000  // Duracion de cada escalon
#define SWEEP_PAUSE_MS      3000  // Pausa con motores en 0% antes de repetir

#define FAN_PWM_PIN         18
#define FAN_DIR_PIN         19
#define MOTOR_LEFT_PWM_PIN  25
#define MOTOR_LEFT_IN1_PIN  27
#define MOTOR_LEFT_IN2_PIN  14
#define MOTOR_RIGHT_PWM_PIN 26
#define MOTOR_RIGHT_IN3_PIN 32
#define MOTOR_RIGHT_IN4_PIN 33

#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_MAX_DUTY_COUNT  1023

static const char *TAG = "PRUEBA_MOTORES";

static void pwm_channel(int gpio, ledc_channel_t ch, uint32_t duty) {
    ledc_channel_config_t c = {
        .gpio_num = gpio, .speed_mode = PWM_MODE, .channel = ch,
        .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0,
        .duty = duty, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void set_motor_duty(uint32_t duty) {
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, duty);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
}

static void set_dir(int dir) { // +1 avance del firmware, -1 retroceso, 0 libre
    gpio_set_level(MOTOR_LEFT_IN1_PIN, dir > 0);
    gpio_set_level(MOTOR_LEFT_IN2_PIN, dir < 0);
    gpio_set_level(MOTOR_RIGHT_IN3_PIN, dir > 0);
    gpio_set_level(MOTOR_RIGHT_IN4_PIN, dir < 0);
}

// Aplica un pulso y devuelve la velocidad integrada de la IMU [m/s] (positiva = +X del sensor)
static float pulso(int dir, const char *nombre) {
    const int dt_ms = 10;
    float v = 0.0f, v_pico = 0.0f, gz_int = 0.0f;
    gpio_set_level(LED_PIN, dir > 0);
    set_dir(dir);
    set_motor_duty(DIR_PULSE_PERCENT * PWM_MAX_DUTY_COUNT / 100);
    for (int t = 0; t < DIR_PULSE_MS + 400; t += dt_ms) {
        if (t >= DIR_PULSE_MS) { set_motor_duty(0); set_dir(0); }
        float ax = 0.0f, gz = 0.0f;
        mpu6050_read_accel_x(&ax);
        mpu6050_read_gyro_z(&gz);
        v += ax * dt_ms * 1e-3f;
        gz_int += gz * dt_ms * 1e-3f;
        if (fabsf(v) > fabsf(v_pico)) v_pico = v;
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(TAG, "%s: velocidad pico IMU = %+.3f m/s (%s del sensor), giro acumulado = %+.1f deg",
             nombre, v_pico, v_pico > 0 ? "+X" : "-X", gz_int);
    return v_pico;
}

static void prueba_direccion(void) {
    gpio_config_t led = { .pin_bit_mask = 1ULL << LED_PIN, .mode = GPIO_MODE_OUTPUT };
    ESP_ERROR_CHECK(gpio_config(&led));
    ESP_LOGW(TAG, "Prueba de direccion: coloque el carro en el piso con espacio. Empieza en %d s.", DIR_WAIT_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(DIR_WAIT_MS));

    if (mpu6050_init() != ESP_OK || mpu6050_calibrate_bias(100) != ESP_OK) {
        ESP_LOGE(TAG, "IMU no disponible: solo se moveran los motores (observe el sentido).");
    }

    float v_av = pulso(+1, "AVANCE del firmware (u>0, LED encendido)");
    vTaskDelay(pdMS_TO_TICKS(DIR_SETTLE_MS));
    float v_re = pulso(-1, "RETROCESO del firmware (u<0, LED apagado)");

    // El observador del carro asume que el avance (u>0) produce aceleracion +X (reduce d)
    if (v_av > 0.0f && v_re < 0.0f) {
        ESP_LOGI(TAG, "RESULTADO: IMU coherente con el firmware (avance = +X). "
                      "Si con el LED encendido el carro se alejo de la compuerta, invierta el sentido de los motores.");
    } else if (v_av < 0.0f && v_re > 0.0f) {
        ESP_LOGW(TAG, "RESULTADO: el avance del firmware acelera hacia -X del sensor: "
                      "motores o IMU estan invertidos respecto al firmware.");
    } else {
        ESP_LOGW(TAG, "RESULTADO: no concluyente (el carro casi no se movio o se movio igual en ambos pulsos).");
    }
    ESP_LOGI(TAG, "Fin de la prueba. Reinicie la placa para repetirla.");
}

void app_main(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << FAN_DIR_PIN) |
                        (1ULL << MOTOR_LEFT_IN1_PIN) | (1ULL << MOTOR_LEFT_IN2_PIN) |
                        (1ULL << MOTOR_RIGHT_IN3_PIN) | (1ULL << MOTOR_RIGHT_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    ledc_timer_config_t t = {
        .speed_mode = PWM_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    // Ventilador apagado
    gpio_set_level(FAN_DIR_PIN, 0);
    pwm_channel(FAN_PWM_PIN, LEDC_CHANNEL_0, 0);

    // Avance: OUT1 y OUT3 en alto (positivo de los motores)
    gpio_set_level(MOTOR_LEFT_IN1_PIN, 1);
    gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_IN3_PIN, 1);
    gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);
    pwm_channel(MOTOR_LEFT_PWM_PIN, LEDC_CHANNEL_1, 0);
    pwm_channel(MOTOR_RIGHT_PWM_PIN, LEDC_CHANNEL_2, 0);

#if TEST_MODE == 1
    prueba_direccion();
    return;
#endif

    while (1) {
        for (int pct = SWEEP_START_PERCENT; pct <= SWEEP_END_PERCENT; pct += SWEEP_STEP_PERCENT) {
            uint32_t duty = (uint32_t)(pct * PWM_MAX_DUTY_COUNT / 100);
            set_motor_duty(duty);
            ESP_LOGI(TAG, "Escalon PWM motores: %3d%% (duty %4lu/1023)", pct, (unsigned long)duty);
            vTaskDelay(pdMS_TO_TICKS(SWEEP_STEP_MS));
        }
        set_motor_duty(0);
        ESP_LOGI(TAG, "Pausa: motores en 0%% durante %d ms, luego se repite el barrido", SWEEP_PAUSE_MS);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_PAUSE_MS));
    }
}
