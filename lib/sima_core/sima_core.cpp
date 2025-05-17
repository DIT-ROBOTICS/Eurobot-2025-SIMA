#include "sima_core.h"
#include "config.h"

#include "VL53L0X_Sensors.h"
#include "motion_control.h"
#include "esp_now_comm.h"

#include <math.h>
#include <ESP32Servo.h>
#include <WebSerial.h>

void IRAM_ATTR stepperCallbackL(void *arg);
void IRAM_ATTR stepperCallbackR(void *arg);
//void IRAM_ATTR checkGoalCallback(void *arg);

esp_timer_handle_t stepperTimerL, stepperTimerR;
//esp_timer_handle_t goalCheckTimer;

VL53L0X_Sensors sensors;
Servo servoL;
Servo servoR;
volatile int start_reach_goal = 0;

float    stepDelayL,       stepDelayR;
bool  accelerationL,    accelerationR,
      decelerationL,    decelerationR,
          rotatingL,        rotatingR, 
      going,
      goingBack,
      reach_goal,
      avoiding;
float x_1,    y_1,
      x_goal, y_goal,
      theta,
      range,
      distanceL,   distanceR,
      mission,  avoidStage,
      adjust;
int step=0, preStep=0, test=1, team=1;
int maxStepDelay = 180, minStepDelay = 55;;


void initSimaCore() {
    stepDelayL = maxStepDelay, stepDelayR = maxStepDelay;
    accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
    rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
    avoiding = false;
    x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;
    distanceL=0, distanceR=0, range=40;
    mission=1, avoidStage=0, adjust=0;
    step=0, preStep=0, test=1, team=1;


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

    // Timers for stepper motors
    esp_timer_create_args_t timerArgsL = {};
    timerArgsL.callback = &stepperCallbackL;
    timerArgsL.name = "StepperTimerL";
    esp_timer_create(&timerArgsL, &stepperTimerL);

    esp_timer_create_args_t timerArgsR = {};
    timerArgsR.callback = &stepperCallbackR;
    timerArgsR.name = "StepperTimerR";
    esp_timer_create(&timerArgsR, &stepperTimerR);

    //check goal timer
    // esp_timer_create_args_t checkGoalArgs = {};
    // checkGoalArgs.callback = &checkGoalCallback;
    // checkGoalArgs.name = "GoalCheck";
    // esp_timer_create(&checkGoalArgs, &goalCheckTimer);

    // Start the timers
    //esp_timer_start_periodic(goalCheckTimer, 200 * 1000);

    // Set 1/8 microstep mode (MS1 = LOW, MS2 = LOW)
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);

    accelerationL=true;
    accelerationR=true;

    // Scan I2C bus and detect connected devices
    Serial.println("\nI2C Scanner Starting...");
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("I2C Device Found at 0x%02X\n", address);
        }
    }
    Serial.println("I2C Scan Done.");

    sensors.begin();

    servoR.write(0);
    servoL.write(0);

}

void setSimaGoal(int num, int team){
    
    if (num==1&&team==1) {
        x_1     = 100;
        y_1     = 1830;
        theta = 338.78;
        x_goal  = 950;
        y_goal  = 1500;
    }
    if (num==1&&team==2) {
        x_1     = 2900;
        y_1     = 1830;
        theta = 201.22;
        x_goal  = 2050;
        y_goal  = 1500;
    }
    if (num==2&&team==1) {
        x_1     = 100;
        y_1     = 1720;
        theta   = 342.86; 
        x_goal  = 1300;
        y_goal  = 1350;
    }
    if (num==2&&team==2) {
        x_1     = 2900;
        y_1     = 1720;
        theta   = 197.14; 
        x_goal  = 1700;
        y_goal  = 1350;
    }
    if (num==3&&team==1) {
        x_1     = 100;
        y_1     = 1600;
        theta   = 356.4;
        x_goal  = 1850;
        y_goal  = 1500;
    }
    if (num==3&&team==2) {
        x_1     = 2900;
        y_1     = 1610;
        theta   = 183.6;
        x_goal  = 1150;
        y_goal  = 1500;
    }
}
void stop() {

    distanceL = 0;
    distanceR = 0;
    accelerationL = 0;
    accelerationR = 0;
    decelerationL = 0;
    decelerationR = 0;
}

void switchcase() {
    if (step != 0) {
        preStep = step;
        x_1 += preStep * lengthPerStep * cos(theta * DEG_TO_RAD);
        y_1 += preStep * lengthPerStep * sin(theta * DEG_TO_RAD);
        step -= preStep;
        //WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
    }
    going = false;
    goingBack = false;
    rotatingL = false;
    rotatingR = false;
}

void IRAM_ATTR stepperCallbackL(void *arg) {
    if (reach_goal) return;
    GPIO.out ^= STEP_BIT_L;
    distanceL -= lengthPerStep;
    if      (going)       step  += 1;
    else if (goingBack)   step  -= 1;
    else if (rotatingL)   theta += rotateAnglePerStep;
    else if (rotatingR)   theta -= rotateAnglePerStep;

    if (accelerationL) {
        stepDelayL-=accRate;
        if (stepDelayL <= minStepDelay || distanceL <= decDistance)
            accelerationL = false;
    }
    // deceleration due to distance
    if (distanceL <= decDistance) {                                                         
        if(!rotatingL || !rotatingR) {
            if (stepDelayL < maxStepDelay) stepDelayL+=decRate;
        }
    }
    // deceleration due to too close to an obstacle
    if (decelerationL) {
        if (stepDelayL < maxStepDelay) stepDelayL+=decRate;
    }
    if (distanceL > 0) {
        esp_timer_start_once(stepperTimerL, stepDelayL);
    } else if (distanceL <= 0) {
        
        if (avoidStage==0)  mission += 0.5;
        if (avoidStage > 0 && adjust == 0) avoidStage += 0.5; 
        if (adjust== 4.5 )adjust += 0.5;        
        switchcase();
    }
}

void IRAM_ATTR stepperCallbackR(void *arg) {
    if (reach_goal) return;
    GPIO.out ^= STEP_BIT_R;
    distanceR -= lengthPerStep;

    if (accelerationR) {
        stepDelayR-=accRate;
        if (stepDelayR <= minStepDelay || distanceR <= decDistance)
            accelerationR = false;
    }
    
    if (distanceR <= decDistance) {
        if (!rotatingL || !rotatingR) {
            if (stepDelayR < maxStepDelay) stepDelayR += decRate;
        }
    }
    if (decelerationR) {
        if (stepDelayR < maxStepDelay) stepDelayR += decRate;
    }
    if (distanceR > 0) {
        esp_timer_start_once(stepperTimerR, stepDelayR);
    } 
}

// void IRAM_ATTR checkGoalCallback(void* arg) {
//     if (!reach_goal) {
//         if(SIMA_NUM==1){
//             if (y_1 <= 1525 &&                                     
//                 y_1 >= (-7.0 / 12.0) * x_1 + 2021.67 &&            
//                 y_1 >= (7.0 / 4.0) * x_1 - 662.5) {
//                     //reach_goal = true;
//                     distanceL = 0;
//                     distanceR = 0;
//                     WebSerial.println("[SIMA-CORE] Goal Reached.");
//             }
//         }//useless

//         if(SIMA_NUM==2){
//             if (x_1>=1300 && x_1<=1700 && y_1>=1300&&y_1<=1500) {
//                     reach_goal = true;
//                     distanceL = 0;
//                     distanceR = 0;
//             }        
//         }

//         if(SIMA_NUM==3){
//             if (y_1 <= 1500 && x_1 * 2 + y_1 >= 5000 && -x_1 * 0.545 + y_1 >= 354.55) {
//                     reach_goal = true;
//                     distanceL = 0;
//                     distanceR = 0;
//             }
//         }
//     }    
// }
void avoidance(){

    if (avoidStage != 1) {
        if (VL53M < 100) {
                stop();
                avoidStage = 1;
                adjust = 1;
        }else if (VL53R < 100 && team == 1) {
                stop();
                avoidStage = 1;
                adjust = 2;
        }else if (VL53L < 100 && team == 2) {
                stop();
                avoidStage = 1;
                adjust = 3;
        }                                    
    }                    
                
    if (avoidStage == 1 ) {
        mission += 10; //never going back
        if (adjust == 1) {
            if(team == 1){
                turnRight(45); 
            }else if(team == 2){
                turnLeft(45); 
            }                        
            adjust = 4.5;
            } else if (adjust == 2) { 
                turnLeft(20);
                adjust = 4.5;
            } else if (adjust == 3) {
                turnRight(20);
                adjust = 4.5;
            } else if (adjust == 5) {
                adjust = 0;
                avoidStage = 2;
            }
    }else if (avoidStage == 2 ) {
            goForward(75);
            avoidStage = 2.5;
    }else if (avoidStage == 3 ) {
            goToTheta(x_goal, y_goal);
            avoidStage = 3.5;
    }else if (avoidStage == 4 ) {
            goToDistance(x_goal, y_goal);
            avoidStage = 4.5;
    }else if (avoidStage == 5 ) {
            reach_goal = 1;
    }
}
void party_time(){

    WebSerial.println("[SIMA-CORE] Timeout reached, stopping motors.");
    stop();
    servoL.write(0);
    servoR.write(0);
    vTaskDelay(1000);
    servoL.write(45);
    servoR.write(45);
    vTaskDelay(1000);    
}

void sima_core_1(void *parameter) {
    unsigned long startTime = 0;
    bool sima_started = false;
    bool sima_timeout = false;
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {

            if (!sima_started) {
                if(espNow.lastMessage.sima_start>0){
                    team = espNow.lastMessage.sima_start;
                }else if(start_reach_goal>0){
                    team = start_reach_goal;
                }
                startTime = millis();
                sima_started = true;
                setSimaGoal(SIMA_NUM, team);
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 10000) {
                    sima_timeout = true;
                    continue;
                }
            }    
            vTaskDelay(1);
            if (!reach_goal&&!sima_timeout) {
                if (mission == 1 ) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    goToDistance(x_goal, y_goal);
                    mission = 1.5;
                }else if (mission == 2 ) {
                    reach_goal=1;
                }
            }
            if (sima_timeout) {
                party_time();
            }
        }    
    }        
}

void sima_core_2(void *parameter) {
    unsigned long startTime = 0;
    bool sima_started = false;
    bool sima_timeout = false;
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {  
            if (!sima_started) {
                if(espNow.lastMessage.sima_start>0){
                    team = espNow.lastMessage.sima_start;
                }else if(start_reach_goal>0){
                    team = start_reach_goal;
                }
                startTime = millis();
                sima_started = true;
                setSimaGoal(SIMA_NUM, team);
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 10000) {
                    sima_timeout = true;
                    continue;
                }
            }
            
                sensors.readSensors();

            //WebSerial.printf("[SIMA-VL53] VL53L=%.2f, VL53M=%.2f, VL53R=%.2f\n", VL53L, VL53M, VL53R);
            if (step != 0) {
                preStep = step;
                x_1 += preStep * lengthPerStep * cos(theta * DEG_TO_RAD); // pi/180
                y_1 += preStep * lengthPerStep * sin(theta * DEG_TO_RAD);
                step -= preStep;
                //WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
            }
                
            if (theta > 360) theta = fmod(theta, 360.0);
            if (theta < 0)   theta += 360;
            //WebSerial.printf("[SIMA-CORE] Current theta=%.2f\n", theta);
            //WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
            if (!reach_goal&&!sima_timeout){ 
                if (mission == 1 ) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    goToDistance(x_goal, y_goal);
                    mission = 1.5;
                }else if (mission == 2 ) {
                    reach_goal=1;
                }
                if(millis() - startTime > 1000){
                avoidance();
                }
            }

            if (sima_timeout) {
                party_time();
            }
        }
    }
}

void sima_core_3(void *parameter) {
    unsigned long startTime = 0;
    bool sima_started = false;
    bool sima_timeout = false;
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {
            
            if (!sima_started) {
                if(espNow.lastMessage.sima_start>0){
                    team = espNow.lastMessage.sima_start;
                }else if(start_reach_goal>0){
                    team = start_reach_goal;
                }
                startTime = millis();
                sima_started = true;
                setSimaGoal(SIMA_NUM, team);
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 10000) {
                    sima_timeout = true;
                    continue;
                }
            }
                sensors.readSensors();
                
                if (step != 0) {
                    preStep = step;
                    x_1 += preStep * lengthPerStep * cos(theta * DEG_TO_RAD); // pi/180
                    y_1 += preStep * lengthPerStep * sin(theta * DEG_TO_RAD);
                    step -= preStep;
                    //WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
                }
                
                if (theta > 360) theta = fmod(theta, 360.0);
                if (theta < 0)   theta += 360;
                // WebSerial.printf("[SIMA-CORE] Current theta=%.2f\n", theta);
                // WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
            if (!reach_goal&&!sima_timeout) {
                if (mission == 1 ) {
                    goToDistance(x_goal, y_goal);
                    mission = 1.5;
                }else if (mission == 2 ) {
                    reach_goal=1;
                }
                avoidance();
            }
            if (sima_timeout) {
                party_time();
            }
        }
    }
}

void sima_core_superstar(void *parameter) {
    unsigned long startTime = 0;
    bool sima_started = false;
    bool sima_timeout = false;
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {
            if (!sima_started) {
                if(espNow.lastMessage.sima_start>0){
                    team = espNow.lastMessage.sima_start;
                }else if(start_reach_goal>0){
                    team = start_reach_goal;
                }
                startTime = millis();
                sima_started = true;
                maxStepDelay = 180;
                minStepDelay = 100;
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 14000) {
                    sima_timeout = true;
                    continue;
                }
            }
                vTaskDelay(1);
            if (!reach_goal&&!sima_timeout) {
                WebSerial.println(mission);
                if (mission == 1 ) {
                    //esp_timer_stop(goalCheckTimer);
                    goForward(1150);
                    mission = 1.5;
                }         
                else if (mission == 2 ) {
                    if(team == 1){
                        turnLeft(90);
                    }
                    else if(team == 2){
                        turnRight(90);
                    }
                    mission = 2.5;
                }
                else if (mission == 3 ) {
                    goBackward(280);
                    mission = 3.5;
                }
                else if (mission == 4 ) {
                    reach_goal=1;
                }
            }
            if (sima_timeout) {
                party_time();
            }     
        }
   
    }        
}