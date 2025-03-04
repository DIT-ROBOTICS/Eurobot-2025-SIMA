#include <Arduino.h>

#define STEP_PIN_L 4   // 連接 TMC2209 STEP 腳位
#define DIR_PIN_L 18    // 連接 TMC2209 DIR 腳位
#define STEP_PIN_R 12   // 連接 TMC2209 STEP 腳位
#define DIR_PIN_R 14    // 連接 TMC2209 DIR 腳位
#define MS1_PIN 5   // 連接 MS1
#define MS2_PIN 16   // 連接 MS2

#define STEPS_PER_REV 6400  // 200 步 * 32 微步 = 6400 microsteps

void setup() {
    Serial.begin(9600);
    pinMode(STEP_PIN_L, OUTPUT);
    pinMode(DIR_PIN_L, OUTPUT);
    pinMode(STEP_PIN_R, OUTPUT);
    pinMode(DIR_PIN_R, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);

    // 設定 1/32 微步模式 (MS1 = HIGH, MS2 = LOW)
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);

    Serial.println("TMC2209 步進馬達控制啟動...");
}

void loop() {

    //foward
    digitalWrite(DIR_PIN_L, LOW);  // 設定方向為順時針
    digitalWrite(DIR_PIN_R, HIGH);  // 設定方向為順時針
    for (int i = 0; i < STEPS_PER_REV*3; i++) {
        digitalWrite(STEP_PIN_L, HIGH);
        delayMicroseconds(15);
        digitalWrite(STEP_PIN_R, HIGH);
        delayMicroseconds(15);
        digitalWrite(STEP_PIN_L, LOW);
        delayMicroseconds(15);
        digitalWrite(STEP_PIN_R, LOW);
        delayMicroseconds(15);
    }

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
