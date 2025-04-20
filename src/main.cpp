#include <Arduino.h>
#include <math.h> 

#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h> 
#include <esp_now.h>

#include <Wire.h>
#include "config.h"
#include "led_control.h"
#include "sima_core.h"
#include "web_interface.h"
#include "VL53L0X_Sensors.h"

extern bool accelerationL, accelerationR;
extern float x_1, y_1, x_goal, y_goal;
extern void IRAM_ATTR stepperCallbackL(void *arg);
extern void IRAM_ATTR stepperCallbackR(void *arg);
extern void IRAM_ATTR checkGoalCallback(void *arg);

TaskHandle_t ledUpdateTaskHandle = NULL;

void setup() {
    Serial.begin(115200);

    initSimaCore();

    // Initialize the web interface components
    webInterface.begin(HOSTNAME);

    // Initialize LED control
    initLED();

    // Create a task to handle LED effects
    xTaskCreatePinnedToCore(
        LEDTask,   // Task function
        "LEDTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        NULL,               // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    // Create a task to handle LED updates
    xTaskCreatePinnedToCore(
        LEDTask,   // Task function
        "LEDUpdateTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &ledUpdateTaskHandle, // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );    
    
    // Create a task to handle the main loop logic
    xTaskCreatePinnedToCore(
        sima_core,   // Task function
        "MainLoopTask",   // Name of the task
        8192,               // Stack size (in bytes)
        NULL,               // Task input parameter
        2,                  // Priority of the task
        NULL,               // Task handle
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    //start point
    x_1=100 ;
    y_1=1600 ;
    x_goal=1900;
    y_goal=1400;
}

void loop() { 
    // Empty as all functionality is now handled in tasks
}
