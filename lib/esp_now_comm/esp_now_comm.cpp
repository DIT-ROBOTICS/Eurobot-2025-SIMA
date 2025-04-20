#include "esp_now_comm.h"

// Create global instance
ESPNowComm espNow;

// Initialize static member variable
ESPNowComm::recv_cb_t ESPNowComm::user_recv_cb = NULL;

ESPNowComm::ESPNowComm() {
    // Initialize lastMessage structure
    memset(&lastMessage, 0, sizeof(struct_message));
}

bool ESPNowComm::begin() {
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    
    // Register receive callback
    esp_now_register_recv_cb(handleRecvData);
    
    Serial.println("ESP-NOW initialized successfully");
    return true;
}

void ESPNowComm::setRecvCallback(recv_cb_t callback) {
    user_recv_cb = callback;
}

bool ESPNowComm::send(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
    // Add peer device (if not already added)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peer_addr, 6);
    if (esp_now_is_peer_exist(peer_addr) == false) {
        esp_now_add_peer(&peerInfo);
    }
    
    // Send data
    esp_err_t result = esp_now_send(peer_addr, data, len);
    return (result == ESP_OK);
}

bool ESPNowComm::sendMessage(const uint8_t *peer_addr, const struct_message &msg) {
    return send(peer_addr, (uint8_t*)&msg, sizeof(struct_message));
}

// Receive callback handler function
void ESPNowComm::handleRecvData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Get source address
    const uint8_t* mac_addr = recv_info->src_addr;
    
    // If data length matches structure size, save to lastMessage
    if (len == sizeof(struct_message)) {
        memcpy(&espNow.lastMessage, data, sizeof(struct_message));
    }
    
    // If user has set a callback function, call it
    if (user_recv_cb != NULL) {
        user_recv_cb(mac_addr, data, len);
    }
}