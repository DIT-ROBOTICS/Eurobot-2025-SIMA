#ifndef ESP_NOW_COMM_H
#define ESP_NOW_COMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// 定義用於SIMA機器人之間通訊的標準消息結構體
typedef struct struct_message {
    int sima_start;
    // 其他您可能需要的欄位
} struct_message;

class ESPNowComm {
public:
    // 構造函數
    ESPNowComm();
    
    // 初始化 ESP-NOW，設置接收回調
    bool begin();
    
    // 設置接收回調函數
    typedef void (*recv_cb_t)(const uint8_t *mac, const uint8_t *data, int len);
    void setRecvCallback(recv_cb_t callback);
    
    // 發送數據到特定地址
    bool send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
    
    // 發送結構化消息
    bool sendMessage(const uint8_t *peer_addr, const struct_message &msg);
    
    // 當前接收到的最後一條消息
    struct_message lastMessage;

private:
    // ESP-NOW 接收回調處理
    static void handleRecvData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    
    // 用戶自定義回調
    static recv_cb_t user_recv_cb;
};

// 全局實例
extern ESPNowComm espNow;

#endif // ESP_NOW_COMM_H