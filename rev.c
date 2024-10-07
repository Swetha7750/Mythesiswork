#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_interface.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "sdkconfig.h"

static const char *TAG = "ESP_NOW_RECEIVER";
uint8_t sender_mac[] = {0x24, 0x0a,0xc4, 0x61, 0x86, 0x84};
  // Replace with actual sender's MAC address

//{0x34, 0x85, 0x18, 0x8D, 0x86, 0xFC}; - sender
//{0x34, 0x85, 0x18, 0x91, 0x37, 0x74}; - rec

void print_mac_address() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Get the MAC address for the station interface

    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


// Data structure to receive
typedef struct {
    char message[32];
} espnow_data_t;

void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // Ensure the MAC address, data, and length are valid

    ESP_LOGI(TAG, "src_address: " MACSTR, MAC2STR(info->src_addr));
    ESP_LOGI(TAG, "des_address: " MACSTR, MAC2STR(info->des_addr));

    if (info->src_addr != NULL && data != NULL && len > 0) {
        // Check if the sender's MAC address matches the expected sender
        if (memcmp(info->src_addr, sender_mac, sizeof(sender_mac)) == 0) {
            espnow_data_t *received_data = (espnow_data_t *)data;
            // Print the received message if it's from the correct sender
            ESP_LOGI(TAG, "Received data from sender: " MACSTR ", Message: %s", 
                     MAC2STR(info->src_addr), received_data->message);
        } else {
            // Warn if the data came from an unexpected MAC address
            ESP_LOGW(TAG, "Received data from unknown sender: " MACSTR, MAC2STR(info->src_addr));
        }
    } else {
        // Log warning if the received data is invalid
        ESP_LOGW(TAG, "Received invalid data");
    }
}

void init_espnow() {
    // Initialize ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*) CONFIG_ESPNOW_PMK));

    // Add peer info
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "peer memory allocation failed");
        esp_now_deinit();
        return;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, 16);
    memcpy(peer->peer_addr, sender_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    // // Add sender peer information
    // //uint8_t sender_mac[] = {0x34, 0x85, 0x18, 0x91, 0x37, 0x74};  // Replace with actual sender's MAC address
    // esp_now_peer_info_t peer_info = {};
    // memcpy(peer_info.peer_addr, sender_mac, 6);
    // peer_info.channel = 1; // Set to the same channel as the receiver
    // peer_info.encrypt = false;

    // esp_err_t add_peer_err = esp_now_add_peer(&peer_info);
    
    // if (add_peer_err == ESP_OK) {
    //     // Check if the added peer's MAC address matches the defined sender_mac
    //     if (memcmp(peer_info.peer_addr, sender_mac, sizeof(sender_mac)) == 0) {
    //         ESP_LOGI(TAG, "Peer added successfully: " MACSTR, MAC2STR(sender_mac));
    //     } else {
    //         ESP_LOGW(TAG, "Peer added, but MAC address does not match sender: " MACSTR, MAC2STR(peer_info.peer_addr));
    //     }
    // } else {
    //     ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(add_peer_err));
    // }

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));  
    ESP_LOGI(TAG, "esp-now receiver init done");
}

void app_main() {
    // Initialize NVS (Non-volatile storage)
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize WiFi in STA mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Initialize ESPNOW
    init_espnow();


    // Keep the receiver running
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to prevent busy-waiting
    }
}
