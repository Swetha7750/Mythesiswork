#include <stdio.h>
#include <string.h>
#include "esp_err.h"
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

static const char *TAG = "ESP_NOW_TRANSMITTER";

// Receiver's MAC Address (Replace with actual receiver MAC address)
uint8_t receiver_mac[] = {0x34, 0x85, 0x18, 0x91, 0x37, 0x74}; 
// uint8_t receiver_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 


void print_mac_address() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Get the MAC address for the station interface

    ESP_LOGI(TAG, "WIFI MAC Address: " MACSTR, MAC2STR(mac));
    

    esp_base_mac_addr_get(mac);
    ESP_LOGI(TAG, "BASE MAC Address: " MACSTR, MAC2STR(mac));
    
}

// Data structure to send
typedef struct {
    char message[32];
} espnow_data_t;

// Callback when data is sent
void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Data sent to: " MACSTR ", Send status: %s", 
            MAC2STR(mac_addr), status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
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
    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(peer->peer_addr, receiver_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);



    // // Add peer information
    // esp_now_peer_info_t peer_info = {};
    // memcpy(peer_info.peer_addr, receiver_mac, 6);
    // peer_info.channel = 1; // Same WiFi channel
    // peer_info.encrypt = false;

    // esp_err_t add_peer_err = esp_now_add_peer(&peer_info);
    // if (add_peer_err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(add_peer_err));
    // } else {
    //     ESP_LOGI(TAG, "Peer added successfully: " MACSTR, MAC2STR(receiver_mac));
    // }


    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb ) );
    ESP_LOGI(TAG, "esp-now sender init done");
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

    // Prepare data to send
    
    // Data structure to receive
    typedef struct {
        char message[32];
    } espnow_data_t;

    espnow_data_t data_to_send;
    strcpy(data_to_send.message, "Hello from Transmitter");
    print_mac_address();
    // Send the data
    while (1) {
        ESP_LOGI(TAG, "Sending data to receiver...");

        esp_err_t send_result = esp_now_send(receiver_mac, (uint8_t *)&data_to_send, sizeof(data_to_send));
        if (send_result == ESP_OK) {
            ESP_LOGI(TAG, "ESP-NOW Send initiated successfully");
        } else {
            
            ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(send_result));
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Send data every 2 seconds
    }
}
