#include "espnow_car.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "ESPNOW_CAR";

static QueueHandle_t s_rx_cmd_queue = NULL;
static uint8_t s_base_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool s_filter_peer = false;

static void espnow_car_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // Con MAC configurada solo se aceptan paquetes del par (evita cruces entre montajes del laboratorio)
    if (s_filter_peer && memcmp(recv_info->src_addr, s_base_mac, 6) != 0) {
        return;
    }
    if (len != sizeof(base_to_car_packet_t)) {
        return;
    }

    const base_to_car_packet_t *pkt = (const base_to_car_packet_t *)data;
    uint16_t expected_ck = protocol_checksum(data, sizeof(base_to_car_packet_t) - sizeof(uint16_t));
    if (pkt->checksum != expected_ck) {
        ESP_EARLY_LOGW(TAG, "Checksum invalido en paquete recibido de la base");
        return;
    }

    // El callback corre en la tarea de Wi-Fi (no ISR): API normal de cola
    if (s_rx_cmd_queue != NULL) {
        xQueueOverwrite(s_rx_cmd_queue, pkt);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void espnow_car_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
#else
static void espnow_car_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
    // Confirmacion no bloqueante de envio
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_EARLY_LOGD(TAG, "Telemetria no confirmada por la base");
    }
}

esp_err_t espnow_car_init(QueueHandle_t rx_cmd_queue, const uint8_t *target_base_mac) {
    s_rx_cmd_queue = rx_cmd_queue;

    if (target_base_mac != NULL) {
        memcpy(s_base_mac, target_base_mac, 6);
    }
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    s_filter_peer = memcmp(s_base_mac, broadcast, 6) != 0;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t own_mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, own_mac));
    ESP_LOGI(TAG, "MAC propia (copiar en protocol_defs.h del otro nodo): %02X:%02X:%02X:%02X:%02X:%02X",
             own_mac[0], own_mac[1], own_mac[2], own_mac[3], own_mac[4], own_mac[5]);
    if (!s_filter_peer) {
        ESP_LOGW(TAG, "MAC del par sin configurar: broadcast sin filtro de origen");
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_car_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_car_recv_cb));

    esp_now_peer_info_t peer_info;
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, s_base_mac, 6);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW Carro inicializado en canal %d hacia MAC %02X:%02X:%02X:%02X:%02X:%02X",
             ESPNOW_WIFI_CHANNEL,
             s_base_mac[0], s_base_mac[1], s_base_mac[2],
             s_base_mac[3], s_base_mac[4], s_base_mac[5]);
    return ESP_OK;
}

esp_err_t espnow_car_send_telemetry(car_to_base_packet_t *telemetry) {
    if (telemetry == NULL) return ESP_ERR_INVALID_ARG;

    telemetry->checksum = protocol_checksum((const uint8_t *)telemetry, sizeof(car_to_base_packet_t) - sizeof(uint16_t));
    return esp_now_send(s_base_mac, (const uint8_t *)telemetry, sizeof(car_to_base_packet_t));
}
