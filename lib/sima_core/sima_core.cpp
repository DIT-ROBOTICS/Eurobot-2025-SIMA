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
void IRAM_ATTR checkGoalCallback(void *arg);

esp_timer_handle_t stepperTimerL, stepperTimerR, goalCheckTimer;

VL53L0X_Sensors sensors;
Servo servoL;
Servo servoR;
volatile bool start_reach_goal = false;

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
        mission,  avoidStage, firstSimaStepStage,
     escape, adjust;
int step=0, preStep=0, test=1, cntDelayL=0, cntDelayR=0;


void initSimaCore() {
    stepDelayL = maxStepDelay, stepDelayR = maxStepDelay;
    accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
    rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
    avoiding = false;
    x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;
    distanceL=0, distanceR=0, range=40;
    mission=1, avoidStage=0, escape=0, adjust=0;
    step=0, preStep=0, test=1;

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

    esp_timer_create_args_t checkGoalArgs = {};
    checkGoalArgs.callback = &checkGoalCallback;
    checkGoalArgs.name = "GoalCheck";
    esp_timer_create(&checkGoalArgs, &goalCheckTimer);

    // Start the timers
    esp_timer_start_periodic(goalCheckTimer, 200 * 1000);

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

void setSimaGoal(int num){
    
    if (num==1) {
        x_1     = 100;
        y_1     = 1710;
        x_goal  = 900;
        y_goal  = 1500;
    }//USELESS
    if (num==2) {
        x_1     = 100;
        y_1     = 1600;
        theta   = 348.2317; 
        x_goal  = 1300;
        y_goal  = 1350;
    }
    if (num==3) {
        x_1     = 100;
        y_1     = 1825;
        x_goal  = 1850;
        y_goal  = 1450;
    }
    if (num==4) {
        x_1     = 100;
        y_1     = 1975;
        x_goal  = 1400;
        y_goal  = 1150;
    }//USELESS
}
void stop() {

    distanceL = 0;
    distanceR = 0;
    accelerationL = 0;
    accelerationR = 0;
    decelerationL = 0;
    decelerationR = 0;
}

void firstSimaStep(int num) {

    if (num == 1) {
        firstSimaStepStage = 1;
    }
    if (num == 2 ) {
        firstSimaStepStage = 1;
    }
    if (num == 3) {
        firstSimaStepStage = -2;
        vTaskDelay(pdMS_TO_TICKS(1800));
        firstSimaStepStage = -1;
    }
    if (num == 4) {
        esp_timer_stop(goalCheckTimer);
        firstSimaStepStage = 1;
    }
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
        
        if (avoidStage==0&&firstSimaStepStage==1)  mission += 0.5;
        if (firstSimaStepStage == 0.5||firstSimaStepStage == -0.5) firstSimaStepStage += 0.5 ;
        if (avoidStage > 0 && escape == 0 && adjust == 0) avoidStage += 0.5; 
        if (escape == 1.5 || escape == 2.5 )                  escape += 0.5;
        if (adjust == 1.5 || adjust== 4.5 )                   adjust += 0.5;        
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

void IRAM_ATTR checkGoalCallback(void* arg) {
    if (!reach_goal) {
        if(SIMA_NUM==1){
            if (y_1 <= 1525 &&                                     
                y_1 >= (-7.0 / 12.0) * x_1 + 2021.67 &&            
                y_1 >= (7.0 / 4.0) * x_1 - 662.5) {
                    //reach_goal = true;
                    distanceL = 0;
                    distanceR = 0;
                    WebSerial.println("[SIMA-CORE] Goal Reached.");
            }
        }//useless

        if(SIMA_NUM==2){
            if (x_1>=1300 && x_1<=1700 && y_1>=1300&&y_1<=1500) {
                    reach_goal = true;
                    distanceL = 0;
                    distanceR = 0;
            }        
        }

        if(SIMA_NUM==3){
            if (y_1 <= 1500 && x_1 * 2 + y_1 >= 5000 && -x_1 * 0.545 + y_1 >= 354.55) {
                    reach_goal = true;
                    distanceL = 0;
                    distanceR = 0;
            }
        }
    }    
}

void sima_core_1(void *parameter) {
    unsigned long startTime = 0;
    bool sima_started = false;
    bool sima_timeout = false;
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {

            if (!sima_started) {
                startTime = millis();
                sima_started = true;
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 14000) {
                    sima_timeout = true;
                    continue;
                }
            }    
            vTaskDelay(1);

            if (!reach_goal&&firstSimaStepStage==1&&!sima_timeout) {
                if (mission == 1 ) {
                    goForward(850);
                    mission = 1.5;
                }         
                else if (mission == 2 ) {
                    vTaskDelay(2000);
                    turnLeft(90);
                    mission = 2.5;
                }
                else if (mission == 3 ) {
                    goBackward(150);
                    mission = 3.5;
                }
                else if (mission == 4 ) {
                    reach_goal=1;
                }
            }
            if (sima_timeout) {
                WebSerial.println("[SIMA-CORE] Timeout reached, stopping motors.");
                stop();
                servoL.write(0);
                servoR.write(0);
                vTaskDelay(1000);
                servoL.write(45);
                servoR.write(45);
                vTaskDelay(1000);
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
                startTime = millis();
                sima_started = true;
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 14000) {
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
            //WebSerial.printf("[SIMA-CORE] Current theta=%.2f\n", theta);
            //WebSerial.printf("[SIMA-CORE] Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
            if (!reach_goal&&firstSimaStepStage==1&&!sima_timeout){ 
                if (mission == 1 ) {
                    goToDistance(x_goal, y_goal);
                    mission = 1.5;
                }else if (mission == 2 ) {
                    reach_goal=1;
                }

                if (avoidStage != 1) {
                    if (VL53M < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 1;
                    }else if (VL53R < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 2;
                    }else if (VL53L < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 3;
                    }
                }
                
                if (avoidStage == 1 ) {
                    mission += 10; //never going back
                    if (adjust == 1) {
                        turnRight(45);
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
                    avoidStage = 0;
                }
            }

            if (sima_timeout) {
                WebSerial.println("[SIMA-CORE] Timeout reached, stopping motors.");
                stop();
                servoL.write(0);
                servoR.write(0);
                vTaskDelay(1000);
                servoL.write(45);
                servoR.write(45);
                vTaskDelay(1000);
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
                startTime = millis();
                sima_started = true;
            }
            if (sima_started && !sima_timeout) {
                if (millis() - startTime > 14000) {
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
            if (!reach_goal&&firstSimaStepStage==1&&!sima_timeout) {

                if (mission == 1 ) {
                    goForward(1625);
                    mission = 1.5;
                }else if (mission == 2 ) {
                    turnLeft(79.32);
                    mission = 2.5;
                }else if (mission == 3 ) {
                    goForward(602);
                    mission = 3.5;
                }


               if (avoidStage != 1) {
                    if (VL53M < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 1;
                    }else if (VL53R < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 2;
                    }else if (VL53L < 100) {
                        stop();
                        avoidStage = 1;
                        adjust = 3;
                    }
                }
                
                if (avoidStage == 1 ) {
                    mission += 10; //never going back
                    if (adjust == 1) {
                        turnRight(45);
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
                    avoidStage = 0;
                }
            }
            if (sima_timeout) {
                WebSerial.println("[SIMA-CORE] Timeout reached, stopping motors.");
                stop();
                servoL.write(0);
                servoR.write(0);
                vTaskDelay(1000);
                servoL.write(45);
                servoR.write(45);
                vTaskDelay(1000);
            }
        }
    }
}

void sima_core_superstar(void *parameter) {
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {
                vTaskDelay(1);
            if (!reach_goal&&firstSimaStepStage==1) {
                WebSerial.println(mission);
                if (mission == 1 ) {
                    goForward(1050);
                    mission = 1.5;
                }         
                else if (mission == 2 ) {
                    turnLeft(90);
                    mission = 2.5;
                }
                else if (mission == 3 ) {
                    goBackward(300);
                    mission = 3.5;
                }
                else if (mission == 4 ) {
                    reach_goal=1;
                }
            }
        }    
    }        
}