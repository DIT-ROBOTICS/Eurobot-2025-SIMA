#ifndef ESP_NOW_COMM_H
#define ESP_NOW_COMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// message structure for ESP-NOW communication
typedef struct struct_message {
    int sima_start;
} struct_message;

class ESPNowComm {
public:
    ESPNowComm();
    
    // Initialize ESP-NOW, setup receive callback
    bool begin();
    
    // Set receive callback function
    typedef void (*recv_cb_t)(const uint8_t *mac, const uint8_t *data, int len);
    void setRecvCallback(recv_cb_t callback);
    
    // Send data to a specific address
    bool send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
    
    // Send structured message
    bool sendMessage(const uint8_t *peer_addr, const struct_message &msg);
    
    // Last message received
    struct_message lastMessage;

private:
    // ESP-NOW receive callback handler
    static void handleRecvData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    
    // User-defined callback
    static recv_cb_t user_recv_cb;
};

// Global instance
extern ESPNowComm espNow;

#endif // ESP_NOW_COMM_H