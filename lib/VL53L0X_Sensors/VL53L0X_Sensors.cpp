#include "VL53L0X_Sensors.h"

float VL53L = 0.0, VL53M = 0.0, VL53R = 0.0;

VL53L0X_Sensors::VL53L0X_Sensors() {
    // Set XSHUT pins as output
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    // Initialize all sensors (turn off first)
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
}

void VL53L0X_Sensors::begin() {
    // Start I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    delay(10);

    // Start first sensor
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    sensor1.init();
    sensor1.setAddress(0x30);
    Serial.println("Sensor 1 initialized and address set to 0x30");

    // Start second sensor
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.init();
    sensor2.setAddress(0x31);
    Serial.println("Sensor 2 initialized and address set to 0x31");

    // Start third sensor
    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.init();
    sensor3.setAddress(0x32);
    Serial.println("Sensor 3 initialized and address set to 0x32");

    // Start continuous ranging mode
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();
}

void VL53L0X_Sensors::readSensors() {

    VL53L=sensor1.readRangeContinuousMillimeters();
    VL53M=sensor2.readRangeContinuousMillimeters();
    VL53R=sensor3.readRangeContinuousMillimeters();
    

}
