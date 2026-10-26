#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MPU6050_I2C_PORT       I2C_NUM_0
#define MPU6050_I2C_SDA_PIN    21
#define MPU6050_I2C_SCL_PIN    22
#define MPU6050_I2C_FREQ_HZ    400000
#define MPU6050_ADDR           0x68

/**
 * @brief Inicializa el puerto I2C y despierta/configura el sensor MPU6050 (rango +-2g).
 * 
 * @return esp_err_t ESP_OK si el sensor respondio correctamente en el bus.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Ejecuta una calibracion rapida en reposo para hallar el sesgo inicial de aceleracion.
 * 
 * @param samples_count Numero de muestras para promediar el cero (ej. 100).
 * @return esp_err_t ESP_OK en caso de exito.
 */
esp_err_t mpu6050_calibrate_bias(uint16_t samples_count);

/**
 * @brief Lee la aceleracion instantanea en el eje longitudinal X [m/s^2].
 * 
 * @param out_ax Puntero donde se almacenara la aceleracion compensada con el sesgo.
 * @return esp_err_t ESP_OK si la lectura I2C fue exitosa.
 */
esp_err_t mpu6050_read_accel_x(float *out_ax);

/**
 * @brief Indica si el sensor esta operativo y calibrado.
 */
bool mpu6050_is_ready(void);

#endif // MPU6050_H
