/*
+------------------------------------------------------------------+
|     ____  __________   ____        __          __  _             |  
|    / __ \/  _/_  __/  / __ \____  / /_  ____  / /_(_)_________   |
|   / / / // /  / /    / /_/ / __ \/ __ \/ __ \/ __/ / ___/ ___/   |
|  / /_/ // /  / /    / _, _/ /_/ / /_/ / /_/ / /_/ / /__(__  )    |
| /_____/___/ /_/    /_/ |_|\____/_.___/\____/\__/_/\___/____/     |
|                   _____ ______  ______                _  __      |
|                  / ___//  _/  |/  /   |              | |/ /      |
|                  \__ \ / // /|_/ / /| |    ______    |   /       |
|                 ___/ // // /  / / ___ |   /_____/   /   |        |
|                /____/___/_/  /_/_/  |_|            /_/|_|        |
|                                                                  |
+------------------------------------------------------------------+
|                   EUROBOT 2025 - SIMA - FRANCE                   |
+------------------------------------------------------------------+
*/                                                              

#include "config.h"
#include "led_control.h"
#include "voltmeter.h"
#include "sima_core.h"
#include "web_interface.h"
#include "VL53L0X_Sensors.h"

extern float x_1, y_1, x_goal, y_goal;

void setup() {
    Serial.begin(115200);

    initSimaCore();
    initLED();
    initVoltmeter();

    // xTaskCreatePinnedToCore(function, "TaskName", stackSize, parameters, priority, taskHandle, coreID);
    // | stackSize:     Size of the stack in bytes
    // | parameters:    Parameters to pass to the task (NULL if none)
    // | priority:      Task priority (0-24, 0 is the lowest)
    // | taskHandle:    Handle to the task (NULL if not needed)
    // | coreID:        Core to run the task on (0 or 1 for ESP32)
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", 4096, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(voltmeterTask, "Voltmeter Task", 4096, NULL, 0, NULL, 0);
    
    // Initialize the web interface components
    webInterface.begin(HOSTNAME);

    if(SIMA_NUM == 1)
    xTaskCreatePinnedToCore(sima_core_1, "SimaCoreTask1", 8192, NULL, 2, NULL, 1);
    else if(SIMA_NUM == 2)
    xTaskCreatePinnedToCore(sima_core_2, "SimaCoreTask1", 8192, NULL, 2, NULL, 1);
    else if(SIMA_NUM == 3 )
    xTaskCreatePinnedToCore(sima_core, "SimaCoreTask", 8192, NULL, 2, NULL, 1);
    else if(SIMA_NUM == 4)
    xTaskCreatePinnedToCore(sima_core_superstar, "SimaCoreTaskSuperstar", 8192, NULL, 2, NULL, 1);

    // Initialize the starting and goal positions
    setSimaGoal(SIMA_NUM);
    firstSimaStep(SIMA_NUM);

}

void loop() {}
