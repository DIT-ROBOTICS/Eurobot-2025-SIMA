#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <NetWizard.h>
#include <ESPDash.h>

#include "config.h"
#include "led_control.h"
#include "esp_now_comm.h"

extern volatile bool start_reach_goal;
extern volatile int mode;
extern Adafruit_NeoPixel strip;

class WebInterface {
public:
    // Constructor and initialization
    WebInterface();
    void begin(const char* hostname);
    
    // Main loop handlers
    void loop();
    
    // Getters for various components
    AsyncWebServer* getServer()    { return &server; }
    NetWizard*      getNetWizard() { return &NW; }
    ESPDash*        getDashboard() { return &dashboard; }
    
    // Tasks for FreeRTOS
    static void webSerialTask   (void *parameter);
    static void elegantOTATask  (void *parameter);
    static void netWizardTask   (void *parameter);
    static void dashTask        (void *parameter);
    
    // Create tasks
    void createTasks();
    
    // Send WebSerial message
    void sendWebSerial(const String& message);
    void sendWebSerialFormat(const char* format, ...);
    
private:
    // Web server components
    AsyncWebServer server;
    NetWizard NW;
    ESPDash dashboard;
    
    // Dashboard cards
    // Card temperature;
    // Card humidity;
    // Card status;
    // Card button;
    // Card slider;
    Card batteryBar;
    
    // Task handles
    TaskHandle_t dashTaskHandle;
    TaskHandle_t webSerialTaskHandle;
    TaskHandle_t elegantOTATaskHandle;
    TaskHandle_t netWizardTaskHandle;
    
    // Make ota_progress_millis static to be accessible from static methods
    static unsigned long ota_progress_millis;
    
    // Callback functions
    static void onOTAStart();
    static void onOTAProgress(size_t current, size_t final);
    static void onOTAEnd(bool success);
    static void onWebSerialMessage(uint8_t *data, size_t len);
    
    // ESP-NOW data receive callback
    static void onESPNowDataReceived(const uint8_t *mac, const uint8_t *data, int len);
    
    // Setup NetWizard callbacks
    void setupNetWizardCallbacks();
};

// Global instance
extern WebInterface webInterface;

#endif // WEB_INTERFACE_H