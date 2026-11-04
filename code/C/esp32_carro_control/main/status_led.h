#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

#define STATUS_LED_PIN 2 // LED integrado de la placa (D2)

typedef enum {
    STATUS_LED_OFF = 0,  // Apagado: sistema funcionando sin errores
    STATUS_LED_BOOT_OK,  // Parpadea STATUS_LED_BOOT_BLINKS veces al terminar el arranque y se apaga
    STATUS_LED_ERROR,    // Parpadeo rapido continuo mientras haya un error
} status_led_mode_t;

#define STATUS_LED_BOOT_BLINKS   3
#define STATUS_LED_BOOT_HALF_MS  150 // medio periodo del parpadeo de arranque
#define STATUS_LED_ERROR_HALF_MS 100 // medio periodo del parpadeo de error (5 Hz)

/**
 * @brief Configura GPIO2 y lanza la tarea que maneja el parpadeo.
 */
esp_err_t status_led_init(void);

/**
 * @brief Cambia el modo del LED. STATUS_LED_BOOT_OK vuelve solo a STATUS_LED_OFF al terminar.
 */
void status_led_set_mode(status_led_mode_t mode);

#endif // STATUS_LED_H
