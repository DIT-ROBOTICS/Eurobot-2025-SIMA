#include "VL53L0X_Sensors.h"

float VL53L = 0.0, VL53M = 0.0, VL53R = 0.0;

VL53L0X_Sensors::VL53L0X_Sensors() {
    // 設定 XSHUT 腳位為輸出
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    // 初始化所有感測器（先關閉）
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
}

void VL53L0X_Sensors::begin() {
    // 啟動 I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    delay(10);

    // 啟動第一個感測器
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    sensor1.init();
    sensor1.setAddress(0x30);

    // 啟動第二個感測器
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.init();
    sensor2.setAddress(0x31);

    // 啟動第三個感測器
    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.init();
    sensor3.setAddress(0x32);

    // 啟動連續測距模式
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();
}

void VL53L0X_Sensors::readSensors() {

    VL53L=sensor1.readRangeContinuousMillimeters();
    VL53M=sensor2.readRangeContinuousMillimeters();
    VL53R=sensor3.readRangeContinuousMillimeters();

}
