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


// Filtro de Kalman de 2 estados [theta, omega] con modelo de velocidad constante.
// A diferencia de un pasa-bajas de 1er orden, sigue rampas sin error de retardo en
// regimen permanente, asi que reduce el ruido sin agregar el atraso que el LQR no modela.
typedef struct {
    float theta;      // [deg]
    float omega;      // [deg/s]
    float P00, P01, P11;
    bool  initialized;
} angle_kalman_t;

static angle_kalman_t s_kf = {0};

static float angle_kalman_update(angle_kalman_t *kf, float z) {
    const float dt = ADC_SAMPLING_PERIOD_US * 1e-6f;

    if (!kf->initialized) {
        kf->theta = z;
        kf->omega = 0.0f;
        kf->P00 = ANGLE_KF_R;
        kf->P01 = 0.0f;
        kf->P11 = 100.0f;
        kf->initialized = true;
        return kf->theta;
    }

    // Prediccion: x = F x, P = F P F' + Q (Q de aceleracion blanca de densidad q)
    const float q = ANGLE_KF_Q_ACCEL;
    const float dt2 = dt * dt, dt3 = dt2 * dt;
    kf->theta += kf->omega * dt;
    float P00 = kf->P00 + dt * (2.0f * kf->P01 + dt * kf->P11) + q * dt3 / 3.0f;
    float P01 = kf->P01 + dt * kf->P11 + q * dt2 / 2.0f;
    float P11 = kf->P11 + q * dt;

    // Correccion con la medicion del potenciometro
    float S  = P00 + ANGLE_KF_R;
    float K0 = P00 / S;
    float K1 = P01 / S;
    float innov = z - kf->theta;
    kf->theta += K0 * innov;
    kf->omega += K1 * innov;
    kf->P00 = (1.0f - K0) * P00;
    kf->P01 = (1.0f - K0) * P01;
    kf->P11 = P11 - K1 * P01;

    return kf->theta;
}

static void adc_sample_task(void *pvParameters) {
    int raw_val = 0;
    while (1) {
        // Espera notificacion del timer hardware de 100 Hz
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Sobremuestreo: promedia ADC_OVERSAMPLES lecturas seguidas (< 1 ms en total)
        int32_t sum = 0;
        int n_ok = 0;
        for (int i = 0; i < ADC_OVERSAMPLES && s_adc1_handle != NULL; i++) {
            if (adc_oneshot_read(s_adc1_handle, POT_ADC_CHANNEL, &raw_val) == ESP_OK) {
                sum += raw_val;
                n_ok++;
            }
        }

        if (n_ok > 0) {
            float raw_avg = (float)sum / (float)n_ok;
            if (raw_avg < 0.0f) raw_avg = 0.0f;
            if (raw_avg > ADC_MAX_COUNTS) raw_avg = ADC_MAX_COUNTS;
            float raw_angle = (raw_avg / ADC_MAX_COUNTS) * ANGLE_MAX_DEG;

            float filt = angle_kalman_update(&s_kf, raw_angle);
            if (filt < 0.0f) filt = 0.0f;
            if (filt > ANGLE_MAX_DEG) filt = ANGLE_MAX_DEG;
            s_last_angle_deg = filt;
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
