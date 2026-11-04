#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MPU6050";

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define STANDARD_GRAVITY         9.80665f
#define ACCEL_SCALE_2G           16384.0f // LSB por 'g' en rango +-2g
#define GYRO_SCALE_250DPS        131.0f   // LSB por deg/s en rango +-250 deg/s

static float s_accel_bias_x = 0.0f;   // Componente de la gravedad en X con el carro en reposo [m/s^2]
static float s_cos_pitch = 1.0f;      // cos de la inclinacion fija del montaje (forma del carro)
static float s_accel_scale = 1.0f;    // Correccion de ganancia del acelerometro (|g| medido -> 9.80665)
static float s_ax_filt = 0.0f;        // Aceleracion longitudinal filtrada [m/s^2]
static float s_gyro_bias_z = 0.0f;    // Sesgo del giroscopio Z en reposo [deg/s]
static bool s_is_ready = false;

// Lee los tres ejes del acelerometro en m/s^2
static esp_err_t read_accel_xyz(float *ax, float *ay, float *az);
// Lee la velocidad angular Z sin compensar [deg/s]
static esp_err_t read_gyro_z_raw(float *gz);

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

    // Diagnostico: escanear el bus I2C y listar los dispositivos que responden con ACK
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Escaneo I2C: dispositivo en 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "Escaneo I2C (SDA:%d, SCL:%d): %d dispositivo(s) encontrado(s)",
             MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN, found);

    // Verificar WHO_AM_I: 0x68 MPU6050, 0x70 MPU6500, 0x71 MPU9250, 0x73 MPU9255.
    // Todos comparten el mapa de registros del acelerometro usado aqui.
    uint8_t who_am_i = 0;
    esp_err_t who_err = read_bytes(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_err != ESP_OK ||
        (who_am_i != 0x68 && who_am_i != 0x70 && who_am_i != 0x71 && who_am_i != 0x73)) {
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

    // 4. Rango del giroscopio +-250 deg/s (GYRO_CONFIG = 0x00): mide el giro del frente (eje Z)
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_GYRO_CONFIG, 0x00));

    ESP_LOGI(TAG, "IMU inicializada con exito (WHO_AM_I=0x%02X, I2C SDA:%d, SCL:%d, Rango: +-2g).",
             who_am_i, MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN);
    s_is_ready = true;
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_bias(uint16_t samples_count) {
    if (!s_is_ready) return ESP_FAIL;

    ESP_LOGI(TAG, "Calibrando sesgo de reposo IMU (%d muestras)... No mover el carro.", samples_count);
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f, sum_gz = 0.0f;
    uint16_t n_ok = 0, n_gz = 0;

    for (uint16_t i = 0; i < samples_count; i++) {
        float ax, ay, az;
        if (read_accel_xyz(&ax, &ay, &az) == ESP_OK) {
            sum_x += ax; sum_y += ay; sum_z += az;
            n_ok++;
        }
        float gz;
        if (read_gyro_z_raw(&gz) == ESP_OK) {
            sum_gz += gz;
            n_gz++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n_ok == 0) return ESP_FAIL;

    // Vector de gravedad en reposo: la forma del carro deja el sensor inclinado un angulo fijo.
    // La componente de g en X se resta como sesgo, y la inclinacion (pitch) se usa para
    // proyectar la aceleracion medida sobre el eje horizontal de avance del carro.
    float gx = sum_x / n_ok, gy = sum_y / n_ok, gz = sum_z / n_ok;
    float pitch = atan2f(gx, sqrtf(gy * gy + gz * gz));
    s_accel_bias_x = gx;
    s_cos_pitch = cosf(pitch);
    if (s_cos_pitch < 0.5f) s_cos_pitch = 0.5f; // montaje con mas de 60 grados: no amplificar ruido
    // Correccion de ganancia: en reposo |g| debe ser 9.80665 m/s^2 (el sensor puede medir +-10 %)
    float g_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    s_accel_scale = (g_norm > 7.0f && g_norm < 13.0f) ? (STANDARD_GRAVITY / g_norm) : 1.0f;
    s_ax_filt = 0.0f;
    s_gyro_bias_z = (n_gz > 0) ? (sum_gz / n_gz) : 0.0f;

    ESP_LOGI(TAG, "Calibracion completada. g en reposo = (%.3f, %.3f, %.3f) m/s^2, |g| = %.3f, "
             "inclinacion del montaje = %.2f deg, a_x_bias = %.4f m/s^2, escala = %.4f, sesgo gyro Z = %.3f deg/s",
             gx, gy, gz, g_norm, pitch * 57.29578f, s_accel_bias_x, s_accel_scale, s_gyro_bias_z);
    return ESP_OK;
}

static esp_err_t read_accel_xyz(float *ax, float *ay, float *az) {
    uint8_t raw[6];
    esp_err_t ret = read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw, 6);
    if (ret != ESP_OK) return ret;
    *ax = ((float)(int16_t)((raw[0] << 8) | raw[1]) / ACCEL_SCALE_2G) * STANDARD_GRAVITY;
    *ay = ((float)(int16_t)((raw[2] << 8) | raw[3]) / ACCEL_SCALE_2G) * STANDARD_GRAVITY;
    *az = ((float)(int16_t)((raw[4] << 8) | raw[5]) / ACCEL_SCALE_2G) * STANDARD_GRAVITY;
    return ESP_OK;
}

esp_err_t mpu6050_read_accel_x(float *out_ax) {
    if (!s_is_ready || out_ax == NULL) return ESP_FAIL;

    float ax, ay, az;
    esp_err_t ret = read_accel_xyz(&ax, &ay, &az);
    if (ret != ESP_OK) {
        return ret;
    }

    // 1. Quitar la gravedad de reposo y proyectar al eje horizontal de avance
    float a_lin = s_accel_scale * (ax - s_accel_bias_x) / s_cos_pitch;

    // 2. Pasa-bajas de 1er orden (quita vibracion de motores y ruido del sensor)
    s_ax_filt += IMU_ACCEL_LPF_ALPHA * (a_lin - s_ax_filt);

    // 3. Zona muerta suave: por debajo del umbral se considera reposo; por encima se resta
    //    el umbral para que la salida sea continua y solo refleje cambios reales
    float a = s_ax_filt;
    if (fabsf(a) < IMU_ACCEL_DEADBAND) {
        a = 0.0f;
    } else {
        a = (a > 0.0f) ? (a - IMU_ACCEL_DEADBAND) : (a + IMU_ACCEL_DEADBAND);
    }
    *out_ax = a;
    return ESP_OK;
}

static esp_err_t read_gyro_z_raw(float *gz) {
    uint8_t raw[2];
    esp_err_t ret = read_bytes(MPU6050_REG_GYRO_ZOUT_H, raw, 2);
    if (ret != ESP_OK) return ret;
    *gz = (float)(int16_t)((raw[0] << 8) | raw[1]) / GYRO_SCALE_250DPS;
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro_z(float *out_gz_dps) {
    if (!s_is_ready || out_gz_dps == NULL) return ESP_FAIL;
    float gz;
    esp_err_t ret = read_gyro_z_raw(&gz);
    if (ret != ESP_OK) return ret;
    *out_gz_dps = gz - s_gyro_bias_z;
    return ESP_OK;
}

bool mpu6050_is_ready(void) {
    return s_is_ready;
}
