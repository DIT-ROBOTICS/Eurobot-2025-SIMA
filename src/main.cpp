#include <Arduino.h>
#include <math.h> 
#include <ESP32Servo.h>
#include "VL53L0X_Sensors.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include "EspNowW.h"

#define STEP_PIN_L 6   
#define DIR_PIN_L 7    
#define STEP_PIN_R 15   
#define DIR_PIN_R 16    
#define MS1_PIN 4   
#define MS2_PIN 5
#define servoPinR 19 //R:servo 1 
#define servoPinL 20 //L:servo 2  

#define wheelCircum  148.188923//47.17*3.1415926 mm
#define lengthPerDegree 0.411636//148.188923/360 mm
#define lengthPerStep 0.0115772596//148.188923/12800 mm
#define wheelDistance 83.7155// mm (by tiral)
#define carCircum 263//(by tiral)
#define STEPS_PER_REV 12800  // 200 步 * 32 微步 = 6400 microsteps, 6400*2
#define rotateAnglePerStep 0.01585

esp_timer_handle_t stepperTimerL, stepperTimerR, VL53Timer;

float stepDelayL = 70, stepDelayR = 70; 
bool accelerationL = false, accelerationR = false, decelerationL = false, decelerationR = false;
bool rotatingL = false, rotatingR = false, going = false, goingBack = false, reach_goal=false;
bool avoiding = false;
float x_1=0, y_1=0, theta=0, x_goal=0, y_goal=0;//sima 1 ; start (150,1800) ; stop(1000,1400)
float distanceL=0, distanceR=0, range=40;
float missionL=1, missionR=1, avoidStageL=0, avoidStageR=0, escape=0, adjust=0;
int step=0,test=1;

VL53L0X_Sensors sensors;
Servo servoL;
Servo servoR;

typedef struct struct_message {
    // char c[32];
  
    int sima_start;
  
  } struct_message;

  // Create a struct_message called myData
  struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print(myData.sima_start);

}

void IRAM_ATTR stepperCallbackL(void *arg){
    
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
    if(distanceL<=110||decelerationL){
        if(!rotatingL||!rotatingR){
        if(stepDelayL<70){
            stepDelayL+=0.005;
        }
            
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
                stepDelayR+=0.005;
            }
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
    going = true; 

}
void goBackward(float distance){

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
    goingBack = true; 

}
void turnLeft(float degree){

    stepDelayL = 70; 
    stepDelayR = 70; 
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL = carCircum*degree/360;
    distanceR = carCircum*degree/360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    rotatingL=true;

}

void turnRight(float degree){

    stepDelayL = 50; 
    stepDelayR = 50; 
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


void setup() {


    Serial.begin(9600);

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

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(9,WIFI_SECOND_CHAN_NONE);
    

    // Init ESP-NOW
    if (esp_now_init() != ERR_OK) {
    Serial.println("Error initializing ESP-NOW");
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
    
    //checking I2C setting
    Serial.println("\nI2C Scanner Starting...");
  
    for (uint8_t address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.print("I2C Device Found at 0x");
        Serial.println(address, HEX);
      }
    }
    Serial.println("Scan Done.");

    //start point
    x_1=150 ;
    y_1=1600 ;
    x_goal=1900;
    y_goal=1400;

}

void loop() {

    //signal from main
    //Serial.println(myData.sima_start);

    while(!reach_goal){
     sensors.readSensors();

    //Serial.println(myData.sima_start);
    // Serial.print("L=");
    // Serial.print(VL53L);
    // Serial.print("  M=");
    // Serial.print(VL53M);
    // Serial.print("  R=");
    // Serial.print(VL53R);
 
    if(step>3000){
        x_1+=3000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=3000*lengthPerStep*sin(theta * 0.01745329);
        step-=3000;
    }
    else if(step>2000){
        x_1+=2000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=2000*lengthPerStep*sin(theta * 0.01745329);
        step-=2000;
    }
    else if(step>1000){
        x_1+=1000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=1000*lengthPerStep*sin(theta * 0.01745329);
        step-=1000;
    }
    else if(step>400){
        x_1+=400*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=400*lengthPerStep*sin(theta * 0.01745329);
        step-=400;
    }  
    else if(step>100){
        x_1+=100*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=100*lengthPerStep*sin(theta * 0.01745329);
        step-=100;
    }    
    else if(step>20){
        x_1+=20*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=20*lengthPerStep*sin(theta * 0.01745329);
        step-=20;
    }
    else if(step>5){
        x_1+=5*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=5*lengthPerStep*sin(theta * 0.01745329);
        step-=5;
    }
    else if(step>0){
        x_1+=lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1+=lengthPerStep*sin(theta * 0.01745329);
        step-=1;
    }
    if(step<-3000){
        x_1-=3000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=3000*lengthPerStep*sin(theta * 0.01745329);
        step+=3000;
    }
    else if(step<-2000){
        x_1-=2000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=2000*lengthPerStep*sin(theta * 0.01745329);
        step+=2000;
    }
    else if(step<-1000){
        x_1-=1000*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=1000*lengthPerStep*sin(theta * 0.01745329);
        step+=1000;
    }
    else if(step<-400){
        x_1-=400*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=400*lengthPerStep*sin(theta * 0.01745329);
        step+=400;
    }  
    else if(step<-100){
        x_1-=100*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=100*lengthPerStep*sin(theta * 0.01745329);
        step+=100;
    }    
    else if(step<-20){
        x_1-=20*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=20*lengthPerStep*sin(theta * 0.01745329);
        step+=20;
    }
    else if(step<-5){
        x_1-=5*lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=5*lengthPerStep*sin(theta * 0.01745329);
        step+=5;
    }
    else if(step>0){
        x_1-=lengthPerStep*cos(theta * 0.01745329);//pi/180
        y_1-=lengthPerStep*sin(theta * 0.01745329);
        step+=1;
    }
    if(theta>360){
        theta-=360;
    }
    if(theta<-360){
        theta+=360;
    }

    // Serial.print(distanceL);
    // Serial.print(" avoid= ");
    // Serial.print(avoidStageL);
    // Serial.print(" escape= ");
    // Serial.print(escape);
    // Serial.print(" adjust= ");
    // Serial.print(adjust);
    // Serial.print(" theta= ");
    // Serial.print(theta);
    // Serial.print("  x= ");
    // Serial.print(x_1);
    // Serial.print("  y= ");
    // Serial.println(y_1);

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
    if(VL53M<250||VL53R<100){
        decelerationL=1;
        decelerationR=1;
    }
    else if(VL53L<100){
        decelerationL=1;
        decelerationR=1;
    }
    else{
        decelerationL=0;
        decelerationR=0;
    }
    if(avoidStageL!=1){
        if(VL53M<150){
            distanceL=0;
            distanceR=0;
            going=false;
            goingBack=false;
            decelerationL=0;
            decelerationR=0;
            avoidStageL=1;
            avoidStageR=1;
            if(VL53R<100){
                escape=1;
            }
        }
        if(VL53R<70){
            distanceL=0;
            distanceR=0;
            going=false;
            goingBack=false;
            decelerationL=0;
            decelerationR=0;
            avoidStageL=1;
            avoidStageR=1;
            adjust=3;
        }
        if(VL53L<70){
            distanceL=0;
            distanceR=0;
            going=false;
            goingBack=false;
            decelerationL=0;
            decelerationR=0;
            avoidStageL=1;
            avoidStageR=1;
            adjust=1;
        }
    }
    if(avoidStageL==1&&avoidStageR==1){
        
        if(escape==0&&adjust==0){
            turnRight(360);        

            if(VL53M>250){
                distanceL=0;
                distanceR=0;
                rotatingL=false;
                rotatingR=false;
                avoidStageL=1.5;
                avoidStageR=1.5;                                                                                                                                                                                                                                                                        

            }
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
            distanceL=0;
            distanceR=0;
            escape=0;
            rotatingL=false;
            rotatingR=false;
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


    if(y_1<1500){
        if(x_1*2+y_1>=5000&&-x_1*0.545+y_1>=354.55){
            reach_goal=1;
        }
    }
}    
 
    distanceL=0;
    distanceR=0;
    servoR.write(0);
    servoL.write(0);
    delay(1000);
    servoR.write(180);
    servoL.write(180);
    delay(1000);

    // while(myData.sima_start!=0){
    // if(missionL==1&&missionR==1){

    //         goFoward(1700);
    //         missionL=1.5;
    //         missionR=1.5;
            
    //     }
    // }



    
}
