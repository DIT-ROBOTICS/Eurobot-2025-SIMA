#include <Arduino.h>

#define STEP_PIN_L 12   // 連接 TMC2209 STEP 腳位
#define DIR_PIN_L 18    // 連接 TMC2209 DIR 腳位
#define STEP_PIN_R 4   // 連接 TMC2209 STEP 腳位
#define DIR_PIN_R 14    // 連接 TMC2209 DIR 腳位
#define MS1_PIN 5   // 連接 MS1
#define MS2_PIN 16   // 連接 MS2

#define wheelCircum  148.188923//47.17*3.1415926 mm
#define lengthPerDegree 0.411636//mm
#define lengthPerStep 0.0115772596

#define STEPS_PER_REV 12800  // 200 步 * 32 微步 = 6400 microsteps, 6400*2

esp_timer_handle_t stepperTimerL, stepperTimerR;
float stepDelayL = 70, stepDelayR = 70; 
bool accelerationL = false, accelerationR = false;
float distanceL=0, distanceR=0;

int go=1;

void IRAM_ATTR stepperCallbackL(void *arg){
    
    digitalWrite(STEP_PIN_L, !digitalRead(STEP_PIN_L));
    distanceL-=0.0115772596;

    if(accelerationL){
        stepDelayL-=0.005;
        
        if(stepDelayL<=16||distanceL<=50){
            accelerationL=false;
        }    
    }
    if(distanceL<=120){
        stepDelayL+=0.005;
    }
    if(distanceL>=0){
        esp_timer_start_once(stepperTimerL, stepDelayL);
    }
}
void IRAM_ATTR stepperCallbackR(void *arg){
    
    digitalWrite(STEP_PIN_R, !digitalRead(STEP_PIN_R));
    distanceR -= 0.0115772596;

    if(accelerationR){
        stepDelayR -= 0.005;
        
        if(stepDelayR <= 16 || distanceR <= 50){
            accelerationR = false;
        }    
    }
    if(distanceR <= 120){
        stepDelayR += 0.005;
    }
    if(distanceR >= 0){
        esp_timer_start_once(stepperTimerR, stepDelayR);
    }
}


void goFoward(float distance){

    distanceL=distance;
    distanceR=distance;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    accelerationL=true;
    accelerationR=true;

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

    esp_timer_create_args_t timerArgsL = {};
    timerArgsL.callback = &stepperCallbackL;
    timerArgsL.name = "StepperTimerL";
    esp_timer_create(&timerArgsL, &stepperTimerL);
 // 啟動 Timer

    esp_timer_create_args_t timerArgsR = {};
    timerArgsR.callback = &stepperCallbackR;
    timerArgsR.name = "StepperTimerR";
    esp_timer_create(&timerArgsR, &stepperTimerR);
    ;

    // 設定 1/32 微步模式 (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);

    accelerationL=true;
    accelerationR=true;
    

}

void loop() {

    if(go){

        goFoward(1000);
        go=0;
    }
  


//     //foward

//     digitalWrite(DIR_PIN_L, LOW);  // 設定方向為順時針
//     digitalWrite(DIR_PIN_R, HIGH);  // 設定方向為順時針
//     if(go){

//     for (int i = 0; i < STEPS_PER_REV*0.5; i++) {
//             digitalWrite(STEP_PIN_L, HIGH);
//             delayMicroseconds(50);
//             digitalWrite(STEP_PIN_R, HIGH);
//             delayMicroseconds(50);
//             digitalWrite(STEP_PIN_L, LOW);
//             delayMicroseconds(50);
//             digitalWrite(STEP_PIN_R, LOW);
//             delayMicroseconds(50);
//     }
//     for (int i = 0; i < STEPS_PER_REV*0.5; i++) {
//         digitalWrite(STEP_PIN_L, HIGH);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_R, HIGH);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_L, LOW);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_R, LOW);
//         delayMicroseconds(35);
// }
//     for (int i = 0; i < STEPS_PER_REV*1.3756; i++) {
//         digitalWrite(STEP_PIN_L, HIGH);
//         delayMicroseconds(15);
//         digitalWrite(STEP_PIN_R, HIGH);
//         delayMicroseconds(15);
//         digitalWrite(STEP_PIN_L, LOW);
//         delayMicroseconds(15);
//         digitalWrite(STEP_PIN_R, LOW);
//         delayMicroseconds(15);
//     }
//     for (int i = 0; i < STEPS_PER_REV*0.5; i++) {
//         digitalWrite(STEP_PIN_L, HIGH);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_R, HIGH);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_L, LOW);
//         delayMicroseconds(35);
//         digitalWrite(STEP_PIN_R, LOW);
//         delayMicroseconds(35);
// }
//     for (int i = 0; i < STEPS_PER_REV*0.5; i++) {
//         digitalWrite(STEP_PIN_L, HIGH);
//         delayMicroseconds(50);
//         digitalWrite(STEP_PIN_R, HIGH);
//         delayMicroseconds(50);
//         digitalWrite(STEP_PIN_L, LOW);
//         delayMicroseconds(50);
//         digitalWrite(STEP_PIN_R, LOW);
//         delayMicroseconds(50);
//     }
//     go=0;
//     }
    // if(go){

    //     for (int i = 0; i < STEPS_PER_REV*3.3756; i++) {
    //         digitalWrite(STEP_PIN_L, HIGH);
    //         delayMicroseconds(35);
    //         digitalWrite(STEP_PIN_R, HIGH);
    //         delayMicroseconds(35);
    //         digitalWrite(STEP_PIN_L, LOW);
    //         delayMicroseconds(35);
    //         digitalWrite(STEP_PIN_R, LOW);
    //         delayMicroseconds(35);
    //     }
    //  go=0;
    // }
    



    // Serial.println("順時針旋轉一圈...");
    // digitalWrite(DIR_PIN_L, HIGH);  // 設定方向為順時針
    // for (int i = 0; i < STEPS_PER_REV*60; i++) {
    //     digitalWrite(STEP_PIN_L, HIGH);
    //     delayMicroseconds(15);
    //     digitalWrite(STEP_PIN_L, LOW);
    //     delayMicroseconds(15);
    // }

    // delay(1000);

    // Serial.println("逆時針旋轉一圈...");
    // digitalWrite(DIR_PIN_L, LOW);
    // for (int i = 0; i < STEPS_PER_REV*5; i++) {
    //     digitalWrite(STEP_PIN_L, HIGH);
    //     delayMicroseconds(15);
    //     digitalWrite(STEP_PIN_L, LOW);
    //     delayMicroseconds(15);
    // }

    // delay(1000);
}
