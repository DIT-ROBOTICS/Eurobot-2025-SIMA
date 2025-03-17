#include <Arduino.h>
#include <math.h> 
#include "VL53L0X_Sensors.h"

#define STEP_PIN_L 6   
#define DIR_PIN_L 7    
#define STEP_PIN_R 15   
#define DIR_PIN_R 16    
#define MS1_PIN 4   
#define MS2_PIN 5  

#define wheelCircum  148.188923//47.17*3.1415926 mm
#define lengthPerDegree 0.411636//148.188923/360 mm
#define lengthPerStep 0.0115772596//148.188923/12800 mm
#define wheelDistance 81.17// mm
#define carCircum 291.57//92.8*3.1415926 mm
#define STEPS_PER_REV 12800  // 200 步 * 32 微步 = 6400 microsteps, 6400*2
#define rotateAnglePerStep 0.0142978163// (360/((291.57/2)/0.0115772596))/2 = 360/12589.33504/2 = 0.02859563263/2=

esp_timer_handle_t stepperTimerL, stepperTimerR, VL53Timer;

float stepDelayL = 70, stepDelayR = 70; 
bool accelerationL = false, accelerationR = false;
bool rotatingL = false, rotatingR = false, going = false;
bool avoiding = false;
float x_1=2850, y_1=1550, theta=180;//sima 1 ; start (2850,1550) ; stop(1700,1250)
float distanceL=0, distanceR=0;
float missionL=1, missionR=1, avoidStageL=0, avoidStageR=0;

VL53L0X_Sensors sensors; 

void IRAM_ATTR stepperCallbackL(void *arg){
    
    digitalWrite(STEP_PIN_L, !digitalRead(STEP_PIN_L));
    distanceL-=lengthPerStep;
    if(going){

        x_1+=lengthPerStep*cos(theta * PI / 180.0);
        y_1+=lengthPerStep*sin(theta * PI / 180.0);

    }
    else if(rotatingL){
        theta+=rotateAnglePerStep;
    }    
    else if(rotatingR){
        theta-=rotateAnglePerStep;
    }

    if(accelerationL){
        stepDelayL-=0.004;
        
        if(stepDelayL<=12||distanceL<=80){
            accelerationL=false;
        }    
    }
    if(distanceL<=80){
        if(!rotatingL||!rotatingR){

        stepDelayL+=0.005;    
        }
    }
    if(distanceL>=0){
        esp_timer_start_once(stepperTimerL, stepDelayL);
    }
    else{
        missionL+=0.5;
        going = false;
        if(avoiding){
            avoidStageL+=0.5;
        }
    }
}
void IRAM_ATTR stepperCallbackR(void *arg){
    
    digitalWrite(STEP_PIN_R, !digitalRead(STEP_PIN_R));
    distanceR -= lengthPerStep;

    if(accelerationR){
        stepDelayR -= 0.004;
        
        if(stepDelayR <= 12 || distanceR <= 80){
            accelerationR = false;
        }    
    }
    if(distanceR <= 80){
        stepDelayR += 0.005;
    }
    if(distanceR >= 0){
        esp_timer_start_once(stepperTimerR, stepDelayR);
    }
    else{
        missionR+=0.5;
        if(avoiding){
            avoidStageR+=0.5;
        }
    }
}

void IRAM_ATTR VL53Callback(void *arg){

    sensors.readSensors();

}


void goFoward(float distance){

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
void turnLeft(float degree){

    stepDelayL = 60; 
    stepDelayR = 60; 
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL = carCircum*degree/360;
    distanceR = carCircum*degree/360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    rotatingL=true;

}

void turnRight(float degree){

    stepDelayL = 60; 
    stepDelayR = 60; 
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, LOW);
    distanceL = carCircum*degree/360;
    distanceR = carCircum*degree/360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    rotatingR=true;

}
void goToTheta(float x_2, float y_2) {

    float dx = x_2 - x_1;
    float dy = y_2 - y_1;

    float thetaGoal = atan2f(dy, dx) * 180 / M_PI;
    Serial.print(thetaGoal);
    Serial.print("  ");
    float thetaDiff = thetaGoal - theta;
    if (thetaDiff > 180) thetaDiff -= 360;
    if (thetaDiff < -180) thetaDiff += 360;
    Serial.println(thetaDiff);
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
    going = true;

}


void setup() {
    Serial.begin(9600);
    pinMode(STEP_PIN_L, OUTPUT);
    pinMode(DIR_PIN_L, OUTPUT);
    pinMode(STEP_PIN_R, OUTPUT);
    pinMode(DIR_PIN_R, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);

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

    // esp_timer_create_args_t timerArgsV = {};
    // timerArgsV.callback = &VL53Callback;
    // timerArgsV.name = "VL53Timer";
    // esp_timer_create(&timerArgsV, &VL53Timer);
    // esp_timer_start_periodic(VL53Timer, 500000);
    

    // 設定 1/32 微步模式 (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);

    accelerationL=true;
    accelerationR=true;

    //sensors.begin();


}

void loop() {

    // if(missionL==1&&missionR==1){

    //     goToTheta(1500,1250);
    //     missionL=1.5;
    //     missionR=1.5;
           
    //  }
    //  if(missionL==2&&missionR==2){

    //     goToDistance(1500,1250);

    //     missionL=2.5;
    //     missionR=2.5;
           
    //  }

    //sensors.readSensors();

    // if(VL53M<100){

    //     distanceL=0;
    //     distanceR=0;

    // }

    // if(avoidStageL==0&&avoidStageR==0){

    // if(VL53M<100){

    //     avoidStageL=1;
    //     avoidStageR=1;
    //     avoiding=true;
        

    //     if(VL53L<150){

    //         turnRight(45);
    //         avoidStageL=1.5;
    //         avoidStageR=1.5;
    //     }
    //     else {
            
    //         turnLeft(45);
    //         avoidStageL=1.5;
    //         avoidStageR=1.5;
    //     } 
    // }
    // }
    // else if (avoidStageL==2&&avoidStageR==2){

    //     goFoward(100);
    //     avoidStageL=2.5;
    //     avoidStageR=2.5;

    // }
    // else if (avoidStageL==3&&avoidStageR==3){

    //     goToTheta(1500,1250);
    //     avoidStageL=3.5;
    //     avoidStageR=3.5;

    // }
    // else if (avoidStageL==4&&avoidStageR==4){

    //     goToDistance(1500,1250);
    //     avoidStageL=0;
    //     avoidStageR=0;
    //     avoiding=false;
    // }

    // Serial.print("stage= ");
    // Serial.print (avoidStageL);
    
    // // Serial.print(VL53L);
    // // Serial.print("  ");
    // // Serial.print(VL53M);
    // // Serial.print("  ");
    // // Serial.println(VL53R);

    // Serial.print("theta= ");
    // Serial.print(theta);
    // Serial.print("x= ");
    // Serial.print(x_1);
    // Serial.print("y= ");
    // Serial.println(y_1);




    if(missionL==1&&missionR==1){

            goFoward(1100);
            missionL=1.5;
            missionR=1.5;
            
        }
    if(missionL==2&&missionR==2){
    
            turnRight(90);
            missionL=2.5;
            missionR=2.5;
            
        }
    if(missionL==3&&missionR==3){
    
        
            goFoward(250);
            missionL=3.5;
            missionR=3.5;
            
        }
    
}
