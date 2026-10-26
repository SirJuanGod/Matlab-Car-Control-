#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define STANDARD_GRAVITY         9.80665f
#define ACCEL_SCALE_2G           16384.0f // LSB por 'g' en rango +-2g

static float s_accel_bias_x = 0.0f;
static bool s_is_ready = false;

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = {reg, val};
    return i2c_master_write_to_device(MPU6050_I2C_PORT, MPU6050_ADDR, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
}

static esp_err_t read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(MPU6050_I2C_PORT, MPU6050_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

esp_err_t mpu6050_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6050_I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(MPU6050_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(MPU6050_I2C_PORT, conf.mode, 0, 0, 0));

    // Verificar WHO_AM_I (esperado 0x68)
    uint8_t who_am_i = 0;
    if (read_bytes(MPU6050_REG_WHO_AM_I, &who_am_i, 1) != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "No se detecto el MPU6050 en I2C (WHO_AM_I = 0x%02X)", who_am_i);
        s_is_ready = false;
        return ESP_FAIL;
    }

    // 1. Despertar el sensor (PWR_MGMT_1 = 0x01: PLL con referencia de giro X)
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_PWR_MGMT_1, 0x01));

    // 2. Filtro paso bajo digital (DLPF ~44 Hz)
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_CONFIG, 0x03));

    // 3. Rango del acelerometro +-2g (ACCEL_CONFIG = 0x00)
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00));

    ESP_LOGI(TAG, "MPU6050 inicializado con exito (I2C SDA:%d, SCL:%d, Rango: +-2g).",
             MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN);
    s_is_ready = true;
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_bias(uint16_t samples_count) {
    if (!s_is_ready) return ESP_FAIL;

    ESP_LOGI(TAG, "Calibrando sesgo de reposo IMU (%d muestras)... No mover el carro.", samples_count);
    float sum_ax = 0.0f;
    uint8_t raw[2];

    for (uint16_t i = 0; i < samples_count; i++) {
        if (read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw, 2) == ESP_OK) {
            int16_t raw_int = (int16_t)((raw[0] << 8) | raw[1]);
            float ax_raw = ((float)raw_int / ACCEL_SCALE_2G) * STANDARD_GRAVITY;
            sum_ax += ax_raw;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_accel_bias_x = sum_ax / (float)samples_count;
    ESP_LOGI(TAG, "Calibracion completada. Sesgo calculado a_x_bias = %.4f m/s^2", s_accel_bias_x);
    return ESP_OK;
}

esp_err_t mpu6050_read_accel_x(float *out_ax) {
    if (!s_is_ready || out_ax == NULL) return ESP_FAIL;

    uint8_t raw[2];
    esp_err_t ret = read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_int = (int16_t)((raw[0] << 8) | raw[1]);
    float ax_raw = ((float)raw_int / ACCEL_SCALE_2G) * STANDARD_GRAVITY;
    *out_ax = ax_raw - s_accel_bias_x; // Compensado con el sesgo calibrado
    return ESP_OK;
}

bool mpu6050_is_ready(void) {
    return s_is_ready;
}
