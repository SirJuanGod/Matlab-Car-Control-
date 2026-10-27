#include "espnow_base.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "ESPNOW_BASE";

static QueueHandle_t s_rx_telemetry_queue = NULL;
static uint8_t s_peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
#else
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
#endif
{
    if (len != sizeof(car_to_base_packet_t)) {
        return;
    }

    const car_to_base_packet_t *pkt = (const car_to_base_packet_t *)data;
    uint16_t expected_ck = protocol_checksum(data, sizeof(car_to_base_packet_t) - sizeof(uint16_t));
    if (pkt->checksum != expected_ck) {
        ESP_EARLY_LOGW(TAG, "Checksum invalido en paquete recibido del carro");
        return;
    }

    // El callback corre en la tarea de Wi-Fi (no ISR): API normal de cola
    if (s_rx_telemetry_queue != NULL) {
        xQueueOverwrite(s_rx_telemetry_queue, pkt);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
#else
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
    // Callback no bloqueante para confirmacion de envio
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_EARLY_LOGD(TAG, "Fallo envio ESP-NOW al carro");
    }
}

esp_err_t espnow_base_init(QueueHandle_t rx_telemetry_queue, const uint8_t *target_peer_mac) {
    s_rx_telemetry_queue = rx_telemetry_queue;

    if (target_peer_mac != NULL) {
        memcpy(s_peer_mac, target_peer_mac, 6);
    }

    // Inicializar NVS si no esta inicializado
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

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Agregar peer
    esp_now_peer_info_t peer_info;
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, s_peer_mac, 6);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW Base inicializado en canal %d hacia MAC %02X:%02X:%02X:%02X:%02X:%02X",
             ESPNOW_WIFI_CHANNEL,
             s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
             s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
    return ESP_OK;
}

esp_err_t espnow_base_send(base_to_car_packet_t *packet) {
    if (packet == NULL) return ESP_ERR_INVALID_ARG;

    // Calcular checksum antes de enviar
    packet->checksum = protocol_checksum((const uint8_t *)packet, sizeof(base_to_car_packet_t) - sizeof(uint16_t));
    return esp_now_send(s_peer_mac, (const uint8_t *)packet, sizeof(base_to_car_packet_t));
}
