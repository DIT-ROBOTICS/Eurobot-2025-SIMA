#include <Arduino.h>
#include <math.h> 

#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h> 
#include <esp_now.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <NetWizard.h>
#include <ESPDash.h>

#include "config.h"
#include "led_control.h"
#include "sima_core.h"

// Add missing extern declarations and includes for unresolved symbols
#include <Wire.h>
#include "VL53L0X_Sensors.h"

extern bool accelerationL, accelerationR;
extern float x_1, y_1, x_goal, y_goal;
extern void IRAM_ATTR stepperCallbackL(void *arg);
extern void IRAM_ATTR stepperCallbackR(void *arg);
extern void IRAM_ATTR checkGoalCallback(void *arg);

AsyncWebServer server(80);
unsigned long ota_progress_millis = 0;

// Initialize ESP-DASH
ESPDash dashboard(&server, "/dashboard", true);    // <--- We initialize ESP-DASH at "/dashboard" URL so that NetWizard logic is not distrupted

/* 
  Dashboard Cards 
  Format - (Dashboard Instance, Card Type, Card Name, Card Symbol(optional) )
*/
Card temperature(&dashboard, TEMPERATURE_CARD, "Temperature", "°C");
Card humidity(&dashboard, HUMIDITY_CARD, "Humidity", "%");
/* 
  Status Card
  Valid Arguments: (ESPDash dashboard, Card Type, const char* name, const char* status (optional) )
*/
Card status(&dashboard, STATUS_CARD, "Test Status", DASH_STATUS_SUCCESS);
/* 
  Button Card
  Valid Arguments: (ESPDash dashboard, Card Type, const char* name)
*/
Card button(&dashboard, BUTTON_CARD, "Test Button");
/* 
  Slider Card
  Valid Arguments: (ESPDash dashboard, Card Type, const char* name, const char* symbol (optional), int min, int max)
*/
Card slider(&dashboard, SLIDER_CARD, "Test Slider", "", 0, 255, 1);



// Initialize NetWizard
NetWizard NW(&server);
// Setup configuration parameters
// NetWizardParameter nw_header(&NW, NW_HEADER, "MQTT");
// NetWizardParameter nw_divider1(&NW, NW_DIVIDER);
// NetWizardParameter nw_mqtt_host(&NW, NW_INPUT, "Host", "", "mqtt.example.com");
// NetWizardParameter nw_mqtt_port(&NW, NW_INPUT, "Port", "", "1883");




typedef struct struct_message {
    // char c[32];
  
    int sima_start;
  
  } struct_message;

  // Create a struct_message called myData
  struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    WebSerial.print(myData.sima_start);

}

void onOTAStart() {
    // Log when OTA has started
    WebSerial.println("OTA update started!");
    // <Add your own code here>
  }
  
  void onOTAProgress(size_t current, size_t final) {
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000) {
      ota_progress_millis = millis();
      WebSerial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
  }
  
  void onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
      WebSerial.println("OTA update finished successfully!");
    } else {
      WebSerial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
  }



TaskHandle_t dashTaskHandle = NULL;

void dashTask(void *parameter) {
    for (;;) {
        /* Update Card Values */
        temperature.update((int)random(0, 50));
        humidity.update((int)random(0, 100));
        
        /* Send Updates to our Dashboard (realtime) */
        dashboard.sendUpdates();

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}

// Define RTOS task handles
TaskHandle_t webSerialTaskHandle = NULL;
TaskHandle_t elegantOTATaskHandle = NULL;
TaskHandle_t netWizardTaskHandle = NULL;
TaskHandle_t ledUpdateTaskHandle = NULL;

// Function to handle WebSerial in a separate task
void webSerialTask(void *parameter) {
    for (;;) {
        WebSerial.loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

// Function to handle ElegantOTA in a separate task
void elegantOTATask(void *parameter) {
    for (;;) {
        ElegantOTA.loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

// Function to handle NetWizard in a separate task
void netWizardTask(void *parameter) {
    for (;;) {
        NW.loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

void setup() {
    Serial.begin(115200);

    initSimaCore();


    // ----------------------------
    // Configure NetWizard Strategy
    // ----------------------------
    // BLOCKING - Connect to WiFi and wait till portal is active
    // (blocks execution after autoConnect)
    // 
    // NON_BLOCKING - Connect to WiFi and proceed while portal is active in background
    // (does not block execution after autoConnect)
    NW.setStrategy(NetWizardStrategy::BLOCKING);

    // Listen for connection status changes
    NW.onConnectionStatus([](NetWizardConnectionStatus status) {
        String status_str = "";

        switch (status) {
        case NetWizardConnectionStatus::DISCONNECTED:
            status_str = "Disconnected";
            break;
        case NetWizardConnectionStatus::CONNECTING:
            status_str = "Connecting";
            break;
        case NetWizardConnectionStatus::CONNECTED:
            status_str = "Connected";
            break;
        case NetWizardConnectionStatus::CONNECTION_FAILED:
            status_str = "Connection Failed";
            break;
        case NetWizardConnectionStatus::CONNECTION_LOST:
            status_str = "Connection Lost";
            break;
        case NetWizardConnectionStatus::NOT_FOUND:
            status_str = "Not Found";
            break;
        default:
            status_str = "Unknown";
        }

        Serial.printf("NW connection status changed: %s\n", status_str.c_str());
        if (status == NetWizardConnectionStatus::CONNECTED) {
        // Local IP
        Serial.printf("Local IP: %s\n", NW.localIP().toString().c_str());
        // Gateway IP
        Serial.printf("Gateway IP: %s\n", NW.gatewayIP().toString().c_str());
        // Subnet mask
        Serial.printf("Subnet mask: %s\n", NW.subnetMask().toString().c_str());
        }
    });

    // Listen for portal state changes
    NW.onPortalState([](NetWizardPortalState state) {
        String state_str = "";

        switch (state) {
        case NetWizardPortalState::IDLE:
            state_str = "Idle";
            break;
        case NetWizardPortalState::CONNECTING_WIFI:
            state_str = "Connecting to WiFi";
            break;
        case NetWizardPortalState::WAITING_FOR_CONNECTION:
            state_str = "Waiting for Connection";
            break;
        case NetWizardPortalState::SUCCESS:
            state_str = "Success";
            break;
        case NetWizardPortalState::FAILED:
            state_str = "Failed";
            break;
        case NetWizardPortalState::TIMEOUT:
            state_str = "Timeout";
            break;
        default:
            state_str = "Unknown";
        }

        Serial.printf("NW portal state changed: %s\n", state_str.c_str());
    });

    NW.onConfig([&]() {
        Serial.println("NW onConfig Received");

        // Print new parameter values
        // Serial.printf("Host: %s\n", nw_mqtt_host.getValueStr().c_str());
        // Serial.printf("Port: %s\n", nw_mqtt_port.getValueStr().c_str());
        return true; // <-- return true to approve request, false to reject
    });

    // Start NetWizard
    NW.autoConnect(HOSTNAME, "");
    
    // Check if configured
    if (NW.isConfigured()) {
        Serial.println("Device is configured");
    } else {
        Serial.println("Device is not configured");
    }

    if (!MDNS.begin(HOSTNAME)) {
        Serial.println("mDNS init failed");
    } else {
        Serial.println("mDNS started");
    }

    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    esp_wifi_set_channel(primaryChan, secondChan);
    WebSerial.print("Channel: ");
    WebSerial.print(primaryChan);

    // Internal rewrite for ESP-DASH dashboard
    server.rewrite("/", "/dashboard").setFilter(ON_STA_FILTER);    // <---  Add this server.rewrite so that we display ESP-DASH at "/" on STA connections
    
    /* Status card updater - can be used anywhere (apart from global scope) */
    status.update("Warning message", DASH_STATUS_WARNING);
    /* Button card callback */
    button.attachCallback([&](int value){
        WebSerial.println("Button Callback Triggered: "+String((value == 1)?"true":"false"));
        /* Button card updater - you need to update the button with latest value upon firing of callback */
        button.update(value);
        /* Send update to dashboard */
        dashboard.sendUpdates();
    });
    /* Slider card callback */
    slider.attachCallback([&](float value){
        Serial.println("Slider Callback Triggered: "+String(value));
        /* Slider card updater - you need to update the slider with latest value upon firing of callback */
        slider.update(value);
        /* Send update to dashboard */
        dashboard.sendUpdates();
    });


    WebSerial.begin(&server);     // Initialize WebSerial

    // Attach a callback function to handle incoming messages
    WebSerial.onMessage([](uint8_t *data, size_t len) {
        WebSerial.printf("Received %lu bytes from WebSerial: ", len);
        Serial.write(data, len);
        WebSerial.println();
        WebSerial.println("Received Data...");
        String d = "";
        for(size_t i = 0; i < len; i++){
            d += char(data[i]);
        }
        WebSerial.println(d);

        // Check if the received data is "1" and set the flag
        if (d == "GO") {
            start_reach_goal = true;
            WebSerial.println("Starting reach_goal loop...");
        }

        if (d == "RESTORE") {
            WebSerial.println("Factory reset command received. Erasing NW configuration...");
            NW.erase();
            delay(1000);
            WebSerial.println("Restarting ESP...");
            ESP.restart();
        }

        if (d == "RESET") {
            WebSerial.println("Reset command received. Restarting ESP...");
            ESP.restart();
        }

        // Parse the received data
        if (d.startsWith("RGB")) {
            int r, g, b;
            if (sscanf(d.c_str(), "RGB %d %d %d", &r, &g, &b) == 3) {
                WebSerial.printf("Setting RGB to R=%d, G=%d, B=%d\n", r, g, b);
                uint32_t color = strip.Color(r, g, b);
                for (int i = 0; i < strip.numPixels(); i++) {
                    strip.setPixelColor(i, color);
                }
                strip.show();
            } else {
                WebSerial.println("Invalid RGB format. Use: RGB <R> <G> <B>");
            }
        } else if (d.startsWith("BRIGHTNESS")) {
            int brightness;
            if (sscanf(d.c_str(), "BRIGHTNESS %d", &brightness) == 1) {
                WebSerial.printf("Setting brightness to %d\n", brightness);
                strip.setBrightness(brightness);
                strip.show();
            } else {
                WebSerial.println("Invalid BRIGHTNESS format. Use: BRIGHTNESS <value>");
            }
        } else if (d.startsWith("MODE")) {
            int modeValue;
            if (sscanf(d.c_str(), "MODE %d", &modeValue) == 1) {
                WebSerial.printf("Setting mode to %d\n", modeValue);
                mode = modeValue;
            } else {
                WebSerial.println("Invalid MODE format. Use: MODE <value>");
            }
        } else {
            WebSerial.println("Unknown command. Use: RGB, BRIGHTNESS, or MODE");
        }
    });

    ElegantOTA.begin(&server);    // Initialize ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);
    // Disable Auto Reboot
    ElegantOTA.setAutoReboot(true);

    server.begin();

    
    // Init ESP-NOW
    if (esp_now_init() != ERR_OK) {
    WebSerial.println("Error initializing ESP-NOW");
    return;
    }
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));


    

    
    //checking I2C setting
    WebSerial.println("\nI2C Scanner Starting...");
  
    for (uint8_t address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        WebSerial.print("I2C Device Found at 0x");
        WebSerial.println(address, HEX);
      }
    }
    WebSerial.println("Scan Done.");



    // Create a task to run ElegantOTA and WebSerial on the second core
    xTaskCreatePinnedToCore(
        dashTask,   // Task function
        "DashTask",   // Name of the task
        10000,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &dashTaskHandle, // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    // Create separate tasks for WebSerial, ElegantOTA, and NetWizard
    xTaskCreatePinnedToCore(
        webSerialTask,   // Task function
        "WebSerialTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &webSerialTaskHandle, // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    xTaskCreatePinnedToCore(
        elegantOTATask,   // Task function
        "ElegantOTATask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &elegantOTATaskHandle, // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    xTaskCreatePinnedToCore(
        netWizardTask,   // Task function
        "NetWizardTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &netWizardTaskHandle, // Task handle
        0                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

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
}
