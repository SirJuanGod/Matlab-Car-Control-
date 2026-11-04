#include "status_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile status_led_mode_t s_mode = STATUS_LED_OFF;
static TaskHandle_t s_task = NULL;

static void status_led_task(void *arg) {
    while (1) {
        status_led_mode_t mode = s_mode;
        if (mode == STATUS_LED_ERROR) {
            gpio_set_level(STATUS_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_ERROR_HALF_MS));
            gpio_set_level(STATUS_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(STATUS_LED_ERROR_HALF_MS));
        } else if (mode == STATUS_LED_BOOT_OK) {
            for (int i = 0; i < STATUS_LED_BOOT_BLINKS && s_mode == STATUS_LED_BOOT_OK; i++) {
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(STATUS_LED_BOOT_HALF_MS));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(STATUS_LED_BOOT_HALF_MS));
            }
            if (s_mode == STATUS_LED_BOOT_OK) s_mode = STATUS_LED_OFF;
        } else {
            gpio_set_level(STATUS_LED_PIN, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // dormir hasta un cambio de modo
        }
    }
}

esp_err_t status_led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << STATUS_LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) return ret;
    gpio_set_level(STATUS_LED_PIN, 0);
    if (xTaskCreate(status_led_task, "status_led", 2048, NULL, 2, &s_task) != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

void status_led_set_mode(status_led_mode_t mode) {
    if (s_mode == mode) return;
    s_mode = mode;
    if (s_task != NULL) xTaskNotifyGive(s_task);
}
