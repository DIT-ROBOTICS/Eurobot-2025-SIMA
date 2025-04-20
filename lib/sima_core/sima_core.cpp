#include "sima_core.h"
#include "VL53L0X_Sensors.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>
#include <WebSerial.h>
#include "motion_control.h"
#include "esp_now_comm.h" // 引入 ESP-NOW 通訊庫

#include <ESP32Servo.h>

// Declare the callback functions at the top of the file
void IRAM_ATTR stepperCallbackL(void *arg);
void IRAM_ATTR stepperCallbackR(void *arg);
void IRAM_ATTR checkGoalCallback(void *arg);

esp_timer_handle_t stepperTimerL, stepperTimerR, goalCheckTimer;

VL53L0X_Sensors sensors;
Servo servoL;
Servo servoR;
volatile bool start_reach_goal = false;

float stepDelayL = 70, stepDelayR = 70; 
bool accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
bool rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
bool avoiding = false;
float x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;//sima 1 ; start (150,1800) ; stop(1000,1400)
float distanceL=0, distanceR=0, range=40;
float missionL=1, missionR=1, avoidStageL=0, avoidStageR=0, escape=0, adjust=0;
int step=0, preStep=0, test=1;

extern VL53L0X_Sensors sensors;
// extern int VL53M, VL53R, VL53L;


void initSimaCore() {
    stepDelayL = 70, stepDelayR = 70; 
    accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
    rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
    avoiding = false;
    x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;//sima 1 ; start (150,1800) ; stop(1000,1400)
    distanceL=0, distanceR=0, range=40;
    missionL=1, missionR=1, avoidStageL=0, avoidStageR=0, escape=0, adjust=0;
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


    //設定 1/32 微步模式 (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);

    accelerationL=true;
    accelerationR=true;

    // 掃描 I2C 總線並檢測連接的設備
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


void sima_core(void *parameter) {
    for (;;) {
        //signal from main
    //WebSerial.println(myData.sima_start);

    // Add WebSerial logs to debug start_reach_goal
    // WebSerial.print(start_reach_goal);
    if (start_reach_goal || espNow.lastMessage.sima_start) { // 添加對 espNow.lastMessage.sima_start 的檢查
        // WebSerial.println("SIMA core: start_reach_goal is true. Entering loop...");

        if (!reach_goal) {
            // WebSerial.println("SIMA core: Not yet reached goal. Processing...");

            sensors.readSensors();
            // WebSerial.println("SIMA core: Sensors read.");

            if(step!=0){
                preStep=step;
                x_1+=preStep*lengthPerStep*cos(theta * DEG_TO_RAD);//pi/180
                y_1+=preStep*lengthPerStep*sin(theta * DEG_TO_RAD);
                step-=preStep;
                WebSerial.printf("SIMA core: Updated position to x_1=%.2f, y_1=%.2f\n", x_1, y_1);
            }
            if(theta>360){
              theta = fmod(theta, 360.0);  
            }
            if (theta < 0) theta += 360;

            WebSerial.printf("SIMA core: Current theta=%.2f\n", theta);
    
            if(missionL==1&&missionR==1){
                WebSerial.println("SIMA core: Executing goToTheta.");
                goToTheta(x_goal, y_goal);
                missionL=1.5;
                missionR=1.5;   
            }
            if(missionL==2&&missionR==2){
                WebSerial.println("SIMA core: Executing goToDistance.");
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
                
                    goForward(250);
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
}
