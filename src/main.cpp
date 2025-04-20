#include <Arduino.h>
#include <math.h> 
#include <ESP32Servo.h>
#include "VL53L0X_Sensors.h"

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

#define STEP_PIN_L 6   
#define DIR_PIN_L 7    
#define STEP_PIN_R 15   
#define DIR_PIN_R 16    
#define MS1_PIN 4   
#define MS2_PIN 5
#define servoPinR 19 //R:servo 1 
#define servoPinL 20 //L:servo 2  

#define wheelCircum  47.17*PI//mm
#define lengthPerStep wheelCircum/12800//mm
#define wheelDistance 85.53// mm (by tiral)
#define carCircum wheelDistance*PI//(by tiral)
#define STEPS_PER_REV 12800  // 200 步 * 32 微步 = 6400 microsteps, 6400*2
#define rotateAnglePerStep  0.0155

esp_timer_handle_t stepperTimerL, stepperTimerR, goalCheckTimer;

float stepDelayL = 70, stepDelayR = 70; 
bool accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
bool rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
bool avoiding = false;
float x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;//sima 1 ; start (150,1800) ; stop(1000,1400)
float distanceL=0, distanceR=0, range=40;
float missionL=1, missionR=1, avoidStageL=0, avoidStageR=0, escape=0, adjust=0;
int step=0, preStep=0, test=1;

VL53L0X_Sensors sensors;
Servo servoL;
Servo servoR;

bool start_reach_goal = false;

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

void IRAM_ATTR stepperCallbackL(void *arg){
    if (reach_goal) return;
    digitalWrite(STEP_PIN_L, !digitalRead(STEP_PIN_L));
    distanceL-=lengthPerStep;
    if(going){
        step+=1;
    }
    if(goingBack){
        step-=1;
    }
    else if(rotatingL){
        theta+=rotateAnglePerStep;
    }    
    else if(rotatingR){
        theta-=rotateAnglePerStep;
    }

    if(accelerationL){
        stepDelayL-=0.004;
        
        if(stepDelayL<=15||distanceL<=110){
            accelerationL=false;
        }    
    }
    if(distanceL<=110){
        if(!rotatingL||!rotatingR){
        if(stepDelayL<70){
            stepDelayL+=0.004;
        }
            
        }
    }
    if(decelerationL){
        if(stepDelayL<70){
            stepDelayL+=0.01;
        }
    }
    if(distanceL>=0){
        esp_timer_start_once(stepperTimerL, stepDelayL);
    }
    else if(distanceL<=0){
        missionL+=0.5;
        going = false;
        goingBack=false;
        rotatingL=false;
        rotatingR=false;
        if(avoidStageL>0&&escape==0){
            if(adjust==0){
               avoidStageL+=0.5; 
            }
        }
        if(escape>0){
            escape+=0.5;
        }
        if(adjust>0){
            adjust+=0.5;
        }
    }
}
void IRAM_ATTR stepperCallbackR(void *arg){
    if (reach_goal) return;
    digitalWrite(STEP_PIN_R, !digitalRead(STEP_PIN_R));
    distanceR -= lengthPerStep;

    if(accelerationR){
        stepDelayR -= 0.004;
        
        if(stepDelayR <= 15|| distanceR <= 110){
            accelerationR = false;
        }    
    }
    if(distanceR <= 110||decelerationR){
        if(!rotatingL||!rotatingR){
            if(stepDelayR<65){
                stepDelayR+=0.004;
            }
    }
    }
    if(decelerationR){
        if(stepDelayR<70){
            stepDelayR+=0.01;
        }
    }
    if(distanceR >= 0){
        esp_timer_start_once(stepperTimerR, stepDelayR);
    }
    else{
        missionR+=0.5;
        if(avoidStageR>0&&escape==0){
            if(adjust==0){
                avoidStageR+=0.5;
            }
            
        }
    }
}
void IRAM_ATTR checkGoalCallback(void* arg) {
    if (!reach_goal) {
        if (y_1 < 1500) {
            if (x_1 * 2 + y_1 >= 5000 && -x_1 * 0.545 + y_1 >= 354.55) {
                reach_goal = true;
                distanceL = 0;
                distanceR = 0;
            }
        }
    }
    
}

void switchcase (){

    if (step != 0 ) {
        preStep = step;
        x_1 += preStep * lengthPerStep * cos(theta * DEG_TO_RAD);
        y_1 += preStep * lengthPerStep * sin(theta * DEG_TO_RAD);
        step -= preStep;
    }
    going=false;
    goingBack=false;
    rotatingL=false;
    rotatingR=false;
}
void stop(){

    distanceL=0;
    distanceR=0;
    decelerationL=0;
    decelerationR=0;
}


void goFoward(float distance){

    switchcase();
    going = true;
    stepDelayL = 70; 
    stepDelayR = 70; 
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL=distance;
    distanceR=distance;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    accelerationL=true;
    accelerationR=true;
 

}
void goBackward(float distance){

    switchcase();    
    goingBack = true;
    stepDelayL = 70; 
    stepDelayR = 70; 
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, LOW);
    distanceL=distance;
    distanceR=distance;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    accelerationL=true;
    accelerationR=true;
}
void turnLeft(float degree){

    switchcase();
    rotatingL=true;
    stepDelayL = 60; 
    stepDelayR = 60; 
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL = carCircum*degree/360;
    distanceR = carCircum*degree/360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
}

void turnRight(float degree){

    switchcase();
    rotatingR=true;
    stepDelayL = 60; 
    stepDelayR = 60; 
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, LOW);
    distanceL = carCircum*degree/360;
    distanceR = carCircum*degree/360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
}
void goToTheta(float x_2, float y_2) {

    float dx = x_2 - x_1;
    float dy = y_2 - y_1;

    float thetaGoal = atan2f(dy, dx) * 180 / M_PI;
    float thetaDiff = thetaGoal - theta;
    if (thetaDiff > 180) thetaDiff -= 360;
    if (thetaDiff < -180) thetaDiff += 360;

    if (thetaDiff > 0) {
        turnLeft(thetaDiff);
    } else {
        turnRight(-thetaDiff);  
    }
}
void goToDistance(float x_2, float y_2){

    float dx = x_2 - x_1;
    float dy = y_2 - y_1;    
    float distance=sqrt(pow(dx,2)+pow(dy,2));
    goFoward(distance);

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

    stepDelayL = 70, stepDelayR = 70; 
    accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
    rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
    avoiding = false;
    x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;//sima 1 ; start (150,1800) ; stop(1000,1400)
    distanceL=0, distanceR=0, range=40;
    missionL=1, missionR=1, avoidStageL=0, avoidStageR=0, escape=0, adjust=0;
    step=0, preStep=0, test=1;

    Serial.begin(115200);

    pinMode(STEP_PIN_L, OUTPUT);
    pinMode(DIR_PIN_L, OUTPUT);
    pinMode(STEP_PIN_R, OUTPUT);
    pinMode(DIR_PIN_R, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);

    servoL.attach(servoPinL);
    servoR.attach(servoPinR);

    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);

    //setting timer
    esp_timer_create_args_t timerArgsL = {};
    timerArgsL.callback = &stepperCallbackL;
    timerArgsL.name = "StepperTimerL";
    esp_timer_create(&timerArgsL, &stepperTimerL);

    esp_timer_create_args_t timerArgsR = {};
    timerArgsR.callback = &stepperCallbackR;
    timerArgsR.name = "StepperTimerR";
    esp_timer_create(&timerArgsR, &stepperTimerR);

    esp_timer_create_args_t checkGoalArgs = {};
    checkGoalArgs.callback = &checkGoalCallback;
    checkGoalArgs.name = "GoalCheck";
    esp_timer_create(&checkGoalArgs, &goalCheckTimer);
    esp_timer_start_periodic(goalCheckTimer, 200 * 1000);

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


    

    //設定 1/32 微步模式 (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);

    accelerationL=true;
    accelerationR=true;

    sensors.begin();

    servoR.write(0);
    servoL.write(0);
    
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
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    // Create separate tasks for WebSerial, ElegantOTA, and NetWizard
    xTaskCreatePinnedToCore(
        webSerialTask,   // Task function
        "WebSerialTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &webSerialTaskHandle, // Task handle
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    xTaskCreatePinnedToCore(
        elegantOTATask,   // Task function
        "ElegantOTATask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &elegantOTATaskHandle, // Task handle
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    xTaskCreatePinnedToCore(
        netWizardTask,   // Task function
        "NetWizardTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &netWizardTaskHandle, // Task handle
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
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
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );

    // Create a task to handle LED updates
    xTaskCreatePinnedToCore(
        LEDTask,   // Task function
        "LEDUpdateTask",   // Name of the task
        4096,               // Stack size (in bytes)
        NULL,               // Task input parameter
        0,                  // Priority of the task
        &ledUpdateTaskHandle, // Task handle
        1                   // Core to run the task on (0 for core 0, 1 for core 1)
    );    
    
    //start point
    x_1=100 ;
    y_1=1600 ;
    x_goal=1900;
    y_goal=1400;

}

void loop() {

    //signal from main
    //WebSerial.println(myData.sima_start);

    if (start_reach_goal) {

        if (!reach_goal) {

            sensors.readSensors();
        
            if(step!=0){
                preStep=step;
                x_1+=preStep*lengthPerStep*cos(theta * DEG_TO_RAD);//pi/180
                y_1+=preStep*lengthPerStep*sin(theta * DEG_TO_RAD);
                step-=preStep;
            }
            if(theta>360){
              theta = fmod(theta, 360.0);  
            }
            if (theta < 0) theta += 360;
    
            if(missionL==1&&missionR==1){
                goToTheta(x_goal, y_goal);
                missionL=1.5;
                missionR=1.5;   
            }
            if(missionL==2&&missionR==2){
                goToDistance(x_goal, y_goal);
                missionL=2.5;
                missionR=2.5;
            }

            if(VL53M<250||VL53R<150){
                decelerationL=1;
                decelerationR=1;
            }
            else if(VL53L<150){
                decelerationL=1;
                decelerationR=1;
            }
            else{
                decelerationL=0;
                decelerationR=0;
                if(stepDelayL>60){
                    accelerationL=1; 
                }
                if(stepDelayR>60){
                    accelerationR=1; 
                }
            }

            if(avoidStageL!=1){
                if(VL53M<150){
                    stop();
                    avoidStageL=1;
                    avoidStageR=1;
                    if(VL53R<100){
                        escape=1;
                    }
                }
                if(VL53R<70){
                    stop();
                    avoidStageL=1;
                    avoidStageR=1;
                    adjust=3;
                }
                if(VL53L<70){
                    stop();
                    avoidStageL=1;
                    avoidStageR=1;
                    adjust=1;
                }
            }
            if(avoidStageL==1&&avoidStageR==1){
                
                if(escape==0&&adjust==0){
                    turnRight(45);
                    adjust=1.5;
                }
                else if(escape==1){
                    goBackward(200);
                    escape=1.5;
                }
                else if(escape==2){
                    turnRight(90);
                    escape=2.5;
                }
                else if(escape==3){
                    escape=0;
                    avoidStageL=2;
                    avoidStageR=2; 
                }
                else if(adjust==1){
                    turnRight(20);
                    adjust=1.5;
                }
                else if(adjust==3){
                    turnLeft(20);
                    adjust=1.5;
                }        
                else if(adjust==2){
                    adjust=0;
                    avoidStageL=2;
                    avoidStageR=2; 
                }
                
            }
            if(avoidStageL==2&&avoidStageR==2){
                
                    goFoward(250);
                    avoidStageL=2.5;
                    avoidStageR=2.5;
                    
            }    
            if(avoidStageL==3&&avoidStageR==3){
                
                    goToTheta(x_goal, y_goal);
                    avoidStageL=3.5;
                    avoidStageR=3.5;
                    
            }
            if(avoidStageL==4&&avoidStageR==4){
                
                    goToDistance(x_goal, y_goal);
                    avoidStageL=0;
                    avoidStageR=0;
                    
            }
         }    
     }
 
    



}
