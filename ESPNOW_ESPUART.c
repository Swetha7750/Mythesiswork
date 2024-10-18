/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"



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


//uart 
static const char *TAGG = "ESP_NOW_TRANSMITTER";

// Receiver's MAC Address (Replace with actual receiver MAC address)
uint8_t receiver_mac[] = {0x34, 0x85, 0x18, 0x91, 0x37, 0x74}; 
// uint8_t receiver_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 



static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0

#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

void print_mac_address() { //espnow
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Get the MAC address for the station interface

    ESP_LOGI(TAGG, "WIFI MAC Address: " MACSTR, MAC2STR(mac));
    

    esp_base_mac_addr_get(mac);
    ESP_LOGI(TAGG, "BASE MAC Address: " MACSTR, MAC2STR(mac));
    
}

// Data structure to send
typedef struct { //espnow
    char message[32];
} espnow_data_t;

// Callback when data is sent
void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) { //espnow
    ESP_LOGI(TAGG, "Data sent to: " MACSTR ", Send status: %s", 
            MAC2STR(mac_addr), status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");

    if(status==ESP_NOW_SEND_SUCCESS){
        ESP_LOGI(TAGG, "Successful in data transmission");
    }
    else if(status !=ESP_NOW_SEND_FAIL){
        ESP_LOGW(TAGG,"ESP_NOW didnt work");
    }
    else{

    ESP_LOGW(TAGG, "Theres a problem, check your code!!!!!");
    }
}

void init_espnow() { //espnow
    // Initialize ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*) CONFIG_ESPNOW_PMK));

    // Add peer info
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAGG, "peer memory allocation failed");
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
    ESP_LOGI(TAGG, "esp-now sender init done");
}

typedef struct {
    float pos0;
    float pos1;
    float pos2;
    
}Floatstruct;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events tha
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %zu", (size_t)event.size); // event.size represents the size
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");

                espnow_data_t data_to_send;
                 // Copy received data to the message structure
                strncpy(data_to_send.message, (const char*)dtmp, sizeof(data_to_send.message) - 1);
                data_to_send.message[sizeof(data_to_send.message) - 1] = '\0'; // Null-terminate the string
                
                // Send the data via ESP-NOW
                ESP_LOGI(TAGG, "Sending data to receiver...");
                esp_err_t send_result = esp_now_send(receiver_mac, (uint8_t *)&data_to_send, sizeof(data_to_send));
                if (send_result == ESP_OK) {
                    ESP_LOGI(TAGG, "ESP-NOW Send initiated successfully");
                } else {
                    ESP_LOGE(TAGG, "Failed to send data: %s", esp_err_to_name(send_result));
                }

            


               /* if(event.size != sizeof(Floatstruct)){
                    int irr = uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);

                    while(irr  != sizeof(float) *3){
                        float xx, yy, zz;
                        
                        memcpy( &xx, dtmp, sizeof(float));
                        memcpy( &yy, dtmp+sizeof(float), sizeof(float));
                        memcpy( &zz, dtmp+sizeof(Floatstruct)*2, sizeof(float));
                        ESP_LOGI(TAG, "x =%f, y= %f, z=%f", xx, yy, zz);
                        memmove(dtmp, dtmp+sizeof(float)*3, irr -sizeof(float)*3);
                        irr -= sizeof(float)*3;
                    
                
                        


                    }

                }*/

                /*if(event.size <= sizeof(Floatstruct)){
                    Floatstruct *received_struct = (Floatstruct *)dtmp;
                    ESP_LOGI(TAG, "NO DATA");   

                
                }else
                {

                    //uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "helo whats up dude : %d", sizeof(Floatstruct));
                    Floatstruct *received_struct = (Floatstruct *)dtmp;
                    ESP_LOGI(TAG, "Received struct data: ");
                    ESP_LOGI(TAG, "pos0: %f", received_struct->pos0);
                    ESP_LOGI(TAG, "pos1: %f", received_struct->pos1);
                
                    ESP_LOGI(TAG, "pos2: %f", received_struct->pos2);
                    
                    

                    
                }*/



                uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(EX_UART_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(EX_UART_NUM);
                } else {
                    uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{


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

    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 8092, NULL, 12, NULL);

    // Initialize ESPNOW
    init_espnow();

    // Prepare data to send
    
    // Data structure to receive
    /*typedef struct {
        char message[32];
    } espnow_data_t;

    espnow_data_t data_to_send;
    strcpy(data_to_send.message, dtmp);
    print_mac_address();
    // Send the data
    while (1) {
        ESP_LOGI(TAGG, "Sending data to receiver...");

        esp_err_t send_result = esp_now_send(receiver_mac, (uint8_t *)&data_to_send, sizeof(data_to_send));
        if (send_result == ESP_OK) {
            ESP_LOGI(TAGG, "ESP-NOW Send initiated successfully");
        } else {
            
            ESP_LOGE(TAGG, "Failed to send data: %s", esp_err_to_name(send_result));
        }*/

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Send data every 2 seconds

    }

