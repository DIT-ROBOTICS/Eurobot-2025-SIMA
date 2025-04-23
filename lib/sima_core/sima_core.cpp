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
        mission,  avoidStage, 
     escape, adjust;
int step=0, preStep=0, test=1;


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

    // Set 1/32 microstep mode (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
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

    if(num==3){
        x_1     = 100;
        y_1     = 1850;
        x_goal  = 1850;
        y_goal  = 1400;
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
    digitalWrite(STEP_PIN_L, !digitalRead(STEP_PIN_L));
    distanceL -= lengthPerStep;
    if      (going)       step  += 1;
    if      (goingBack)   step  -= 1;
    else if (rotatingL)   theta += rotateAnglePerStep;
    else if (rotatingR)   theta -= rotateAnglePerStep;

    if (accelerationL) {
        stepDelayL -= 0.004;
        if(stepDelayL <= minStepDelay || distanceL <= 110) accelerationL=false;
    }
    // deceleration due to distance
    if (distanceL <= 110) {                                                         
        if(!rotatingL || !rotatingR) {
            if (stepDelayL < maxStepDelay) stepDelayL+=0.004;
        }
    }
    // deceleration due to too close to an obstacle
    if (decelerationL) {
        if (stepDelayL < maxStepDelay) stepDelayL+=0.01;
    }
    if (distanceL > 0) {
        esp_timer_start_once(stepperTimerL, stepDelayL);
    } else if (distanceL <= 0) {
        
        if (avoidStage==0)  mission += 0.5;
        if (avoidStage > 0 && escape == 0 && adjust == 0) avoidStage += 0.5; 
        if (escape == 1.5 || escape == 2.5 )                  escape += 0.5;
        if (adjust == 1.5)                                    adjust += 0.5;        
        switchcase();
    }
}

void IRAM_ATTR stepperCallbackR(void *arg) {
    if (reach_goal) return;
    digitalWrite(STEP_PIN_R, !digitalRead(STEP_PIN_R));
    distanceR -= lengthPerStep;

    if (accelerationR) {
        stepDelayR -= 0.004;
        if (stepDelayR <= minStepDelay || distanceR <= 110) accelerationR = false;
    }
    if (distanceR <= 110) {
        if (!rotatingL || !rotatingR) {
            if (stepDelayR < maxStepDelay) stepDelayR += 0.004;
        }
    }
    if (decelerationR) {
        if (stepDelayR < maxStepDelay) stepDelayR += 0.01;
    }
    if (distanceR > 0) {
        esp_timer_start_once(stepperTimerR, stepDelayR);
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

void stop() {
    distanceL = 0;
    distanceR = 0;
    accelerationL = 0;
    accelerationR = 0;
    decelerationL = 0;
    decelerationR = 0;
}

void sima_core(void *parameter) {
    for (;;) {
        if (start_reach_goal || espNow.lastMessage.sima_start) {
            if (!reach_goal) {
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
        
                if (mission == 1 ) {
                    WebSerial.println("[SIMA-CORE] Executing goToTheta.");
                    goToTheta(x_goal, y_goal);
                    mission = 1.5;
                }         
                else if (mission == 2 ) {
                    WebSerial.println("[SIMA-CORE] Executing goToDistance.");
                    goToDistance(x_goal, y_goal);
                    mission = 2.5;
                }

                if (VL53M < 250 || VL53R < 150) {
                    decelerationL = 1;
                    decelerationR = 1;
                } 
                else if( VL53L < 150){
                    decelerationL = 1;
                    decelerationR = 1;
                }
                else {
                    decelerationL = 0;
                    decelerationR = 0;
                    if (stepDelayL > minStepDelay) accelerationL = 1;
                    if (stepDelayR > minStepDelay) accelerationR = 1;
                }

                if (avoidStage != 1) {
                    if (VL53M < 150) {
                        stop();
                        avoidStage = 1;
                        if (VL53R < 100) escape = 1;
                    }
                    
                    if (VL53R < 70) {
                        stop();
                        avoidStage = 1;
                        adjust = 3;
                    }
                    
                    if (VL53L < 70) {
                        stop();
                        avoidStage = 1;
                        adjust = 1;
                    }
                }
                
                if (avoidStage == 1 ) {
                    mission += 10; //never going back
                    if (escape == 0 && adjust == 0) {
                        turnRight(45);
                        adjust = 1.5;
                    } else if (escape == 1) {
                        goBackward(200);
                        escape = 1.5;
                    } else if (escape == 2) {
                        turnRight(90);
                        escape = 2.5;
                    } else if (escape == 3) {
                        escape = 0;
                        avoidStage = 2;
                    } else if (adjust == 1) {
                        turnRight(20);
                        adjust = 1.5;
                    } else if (adjust == 3) {
                        turnLeft(20);
                        adjust = 1.5;
                    } else if (adjust == 2) {
                        adjust = 0;
                        avoidStage = 2;
                    }
                }
                
                if (avoidStage == 2 ) {
                    goForward(250);
                    avoidStage = 2.5;
                }
                
                if (avoidStage == 3 ) {
                    goToTheta(x_goal, y_goal);
                    avoidStage = 3.5;
                }
                
                if (avoidStage == 4 ) {
                    goToDistance(x_goal, y_goal);
                    avoidStage = 0;
                }
            }
        }
    }
}
