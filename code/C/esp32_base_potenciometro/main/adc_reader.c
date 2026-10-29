#include "adc_reader.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"

static const char *TAG = "ADC_READER";

static esp_timer_handle_t s_adc_timer = NULL;
static float s_last_angle_deg = 0.0f;
static TaskHandle_t s_adc_task_handle = NULL;

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

static float s_filtered_angle = 0.0f;
static bool s_first_sample = true;

static void adc_sample_task(void *pvParameters) {
    int raw_val = 0;
    while (1) {
        // Espera notificacion del timer hardware de 100 Hz
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bool read_ok = (s_adc1_handle != NULL) &&
                       (adc_oneshot_read(s_adc1_handle, POT_ADC_CHANNEL, &raw_val) == ESP_OK);

        if (read_ok) {
            if (raw_val < 0) raw_val = 0;
            if (raw_val > 4095) raw_val = 4095;
            float raw_angle = ((float)raw_val / ADC_MAX_COUNTS) * ANGLE_MAX_DEG;
            if (raw_angle < 0.0f) raw_angle = 0.0f;
            if (raw_angle > ANGLE_MAX_DEG) raw_angle = ANGLE_MAX_DEG;

            // Filtro pasa-bajas IIR de 1er orden (alfa=0.25 a 100 Hz):
            // Elimina el ruido analogico de la escobilla mecanica del potenciometro
            // sin generar retardo perceptible en el lazo de control.
            if (s_first_sample) {
                s_filtered_angle = raw_angle;
                s_first_sample = false;
            } else {
                s_filtered_angle = (0.25f * raw_angle) + (0.75f * s_filtered_angle);
            }

            s_last_angle_deg = s_filtered_angle;
        }
    }
}

static void adc_timer_callback(void *arg) {
    // esp_timer ejecuta este callback en su propia tarea (no ISR): API normal, no FromISR
    if (s_adc_task_handle != NULL) {
        xTaskNotifyGive(s_adc_task_handle);
    }
}

esp_err_t adc_reader_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, POT_ADC_CHANNEL, &chan_config));

    // Crear tarea de lectura de ADC con prioridad alta
    xTaskCreatePinnedToCore(adc_sample_task, "adc_sample_task", 2560, NULL, 5, &s_adc_task_handle, 1);

    // Configurar temporizador de alta resolucion esp_timer para 100 Hz (10 ms)
    const esp_timer_create_args_t timer_args = {
        .callback = &adc_timer_callback,
        .name = "adc_sample_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_adc_timer));

    ESP_LOGI(TAG, "ADC Inicializado en GPIO34 (12 bits, periodo 10 ms / 100 Hz).");
    return ESP_OK;
}

esp_err_t adc_reader_start(void) {
    if (s_adc_timer == NULL) return ESP_FAIL;
    return esp_timer_start_periodic(s_adc_timer, ADC_SAMPLING_PERIOD_US);
}

esp_err_t adc_reader_stop(void) {
    if (s_adc_timer == NULL) return ESP_FAIL;
    return esp_timer_stop(s_adc_timer);
}

float adc_reader_get_last_angle(void) {
    return s_last_angle_deg;
}
