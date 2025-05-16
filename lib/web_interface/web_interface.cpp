#include "web_interface.h"
#include "voltmeter.h"

// Create global instance
WebInterface webInterface;

// Create a variable to access ESP-NOW messages (for backward compatibility)
static struct_message* myDataPtr = &espNow.lastMessage;

// Initialize static member variable
unsigned long WebInterface::ota_progress_millis = 0;

WebInterface::WebInterface() : 
    server(80),
    NW          (&server),
    dashboard   (&server, "/dashboard", true),
    // temperature (&dashboard, TEMPERATURE_CARD, "Temperature", "°C"),
    // humidity    (&dashboard, HUMIDITY_CARD,    "Humidity", "%"),
    // status      (&dashboard, STATUS_CARD,      "Test Status", DASH_STATUS_SUCCESS),
    // button      (&dashboard, BUTTON_CARD,      "Test Button"),
    // slider      (&dashboard, SLIDER_CARD,      "Test Slider", "", 0, 255, 1),
    batteryBar  (&dashboard, PROGRESS_CARD,    "Battery", "%", 0, 255),
    dashTaskHandle      (NULL),
    webSerialTaskHandle (NULL),
    elegantOTATaskHandle(NULL),
    netWizardTaskHandle (NULL)
{
    // Constructor
}

void WebInterface::begin(const char* hostname) {
    // Configure NetWizard
    NW.setStrategy(NetWizardStrategy::BLOCKING);
    setupNetWizardCallbacks();
    NW.autoConnect(hostname, "");
    
    // Check if configured
    Serial.println(NW.isConfigured() ? "Device is configured" : "Device is not configured");

    // Start MDNS
    if (!MDNS.begin(hostname)) {
        Serial.println("mDNS init failed");
    } else {
        Serial.println("mDNS started");
    }

    // WiFi channel configuration
    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    esp_wifi_set_channel(primaryChan, secondChan);
    
    // Internal rewrite for ESP-DASH dashboard
    server.rewrite("/", "/dashboard").setFilter(ON_STA_FILTER);
    
    // Setup Dashboard
    // status.update("Warning message", DASH_STATUS_WARNING);
    
    // Button card callback
    // button.attachCallback([this](int value) {
    //     sendWebSerialFormat("Button Callback Triggered: %s", (value == 1) ? "true" : "false");
    //     button.update(value);
    //     dashboard.sendUpdates();
    // });
    
    // Slider card callback
    // slider.attachCallback([this](float value) {
    //     Serial.printf("Slider Callback Triggered: %f\n", value);
    //     slider.update(value);
    //     dashboard.sendUpdates();
    // });

    // Initialize WebSerial
    WebSerial.begin(&server);
    WebSerial.onMessage(onWebSerialMessage);

    // Initialize ElegantOTA
    ElegantOTA.begin(&server);
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);
    ElegantOTA.setAutoReboot(true);

    // Start the server
    server.begin();
    
    // Initialize ESP-NOW communication library
    if (!espNow.begin()) {
        sendWebSerial("Error initializing ESP-NOW");
        return;
    }
    
    // Set ESP-NOW receive callback
    espNow.setRecvCallback(onESPNowDataReceived);
    
    // Create all the required tasks
    createTasks();
}

// ESP-NOW data receive callback handler
void WebInterface::onESPNowDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // Data is already in espNow.lastMessage, so it can be used directly
    WebSerial.print(myDataPtr->sima_start);
}

void WebInterface::createTasks() {
    // Create tasks for each component
    xTaskCreatePinnedToCore(
        dashTask,
        "DashTask",
        10000,
        NULL,
        0,
        &dashTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        webSerialTask,
        "WebSerialTask",
        4096,
        NULL,
        0,
        &webSerialTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        elegantOTATask,
        "ElegantOTATask",
        4096,
        NULL,
        0,
        &elegantOTATaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        netWizardTask,
        "NetWizardTask",
        4096,
        NULL,
        0,
        &netWizardTaskHandle,
        0
    );
}

void WebInterface::loop() {
    // This function is not needed as everything is handled in tasks
}

void WebInterface::setupNetWizardCallbacks() {
    // Listen for connection status changes
    NW.onConnectionStatus([](NetWizardConnectionStatus status) {
        String status_str = "";

        switch (status) {
        case NetWizardConnectionStatus::DISCONNECTED:
            status_str = "Disconnected";
            mode = WIFI_DISCONNECTED;
            break;
        case NetWizardConnectionStatus::CONNECTING:
            status_str = "Connecting";
            break;
        case NetWizardConnectionStatus::CONNECTED:
            status_str = "Connected";
            mode = DEFAULT_MODE;
            break;
        case NetWizardConnectionStatus::CONNECTION_FAILED:
            status_str = "Connection Failed";
            mode = WIFI_DISCONNECTED;
            break;
        case NetWizardConnectionStatus::CONNECTION_LOST:
            status_str = "Connection Lost";
            mode = WIFI_DISCONNECTED;
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
            Serial.printf("Local IP: %s\n", webInterface.getNetWizard()->localIP().toString().c_str());
            // Gateway IP
            Serial.printf("Gateway IP: %s\n", webInterface.getNetWizard()->gatewayIP().toString().c_str());
            // Subnet mask
            Serial.printf("Subnet mask: %s\n", webInterface.getNetWizard()->subnetMask().toString().c_str());
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
            // If waiting for connection in AP mode, set LED to WIFI_WIZARD mode
            mode = WIFI_WIZARD;
            break;
        case NetWizardPortalState::SUCCESS:
            state_str = "Success";
            // On successful configuration, reset to default LED mode
            mode = DEFAULT_MODE;
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

    // Config callback
    NW.onConfig([&]() {
        Serial.println("NW onConfig Received");
        return true; // Approve request
    });
}

void WebInterface::webSerialTask(void *parameter) {
    for (;;) {
        WebSerial.loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

void WebInterface::elegantOTATask(void *parameter) {
    for (;;) {
        ElegantOTA.loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

void WebInterface::netWizardTask(void *parameter) {
    for (;;) {
        webInterface.getNetWizard()->loop();
        vTaskDelay(1); // Yield to other tasks
    }
}

void WebInterface::dashTask(void *parameter) {
    for (;;) {
        /* Update Battery Voltage Progress Bar by mapping 9-10.95V to 0-255 range */
        const float minVoltage = 9.0;
        const float maxVoltage = 10.95;
        
        // Constrain voltage to defined range and map to 0-255
        float constrainedVoltage = constrain(Vbattf, minVoltage, maxVoltage);
        int batteryLevel = (int)((constrainedVoltage - minVoltage) * 255.0 / (maxVoltage - minVoltage));
        
        webInterface.batteryBar.update(batteryLevel, "%");
        
        /* Send Updates to our Dashboard (realtime) */
        webInterface.getDashboard()->sendUpdates();

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}

void WebInterface::onOTAStart() {
    // Log when OTA has started
    WebSerial.println("OTA update started!");
    // Set LED mode to UPDATING
    mode = UPDATING;
}

void WebInterface::onOTAProgress(size_t current, size_t final) {
    // Log every 1 second
    if (millis() - WebInterface::ota_progress_millis > 1000) {
        WebInterface::ota_progress_millis = millis();
        WebSerial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void WebInterface::onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
        WebSerial.println("OTA update finished successfully!");
    } else {
        WebSerial.println("There was an error during OTA update!");
    }
}

void WebInterface::onWebSerialMessage(uint8_t *data, size_t len) {
    // WebSerial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    // WebSerial.println();
    // WebSerial.println("Received Data...");
    
    String d = "";
    for(size_t i = 0; i < len; i++){
        d += char(data[i]);
    }
    WebSerial.println(d);

    // Check commands
    if (d == "GO1") {
        start_reach_goal = 1;
        WebSerial.println("SIMA GO1 command received. Starting SIMA...");
    }else if (d == "GO2") {
        start_reach_goal = 2;
        WebSerial.println("SIMA GO2 command received. Starting SIMA...");
    }else if (d == "RESTORE") {
        WebSerial.println("Factory reset command received. Erasing NW configuration...");
        webInterface.getNetWizard()->erase();
        delay(1000);
        WebSerial.println("Restarting ESP...");
        ESP.restart();
    } else if (d == "RESET") {
        WebSerial.println("Reset command received. Restarting ESP...");
        ESP.restart();
    } else if (d.startsWith("MODE")) {
        int modeValue;
        if (sscanf(d.c_str(), "MODE %d", &modeValue) == 1) {
            WebSerial.printf("Setting mode to %d\n", modeValue);
            mode = modeValue;
        } else {
            WebSerial.println("Invalid MODE format. Use: MODE <value>");
        }
    } else if (d == "INFO") {
        WebSerial.printf("Battery Voltage: %.2f V\n", Vbattf);
    } else {
        WebSerial.println("Unknown command. ");
    }
}

void WebInterface::sendWebSerial(const String& message) {
    WebSerial.println(message);
}

void WebInterface::sendWebSerialFormat(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    WebSerial.println(buffer);
}