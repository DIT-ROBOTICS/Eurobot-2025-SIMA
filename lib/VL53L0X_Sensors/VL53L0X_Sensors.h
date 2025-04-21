#ifndef VL53L0X_SENSORS_H
#define VL53L0X_SENSORS_H

#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"

extern float VL53L, VL53M, VL53R;

class VL53L0X_Sensors {
public:
    VL53L0X sensor1, sensor2, sensor3;
    
    // XSHUT GPIO pins
    const int XSHUT1 = XSHUT1_PIN;
    const int XSHUT2 = XSHUT2_PIN;
    const int XSHUT3 = XSHUT3_PIN;

    // I2C pins
    const int SDA_PIN = VL53L0X_SDA_PIN;
    const int SCL_PIN = VL53L0X_SCL_PIN;

    VL53L0X_Sensors();
    void begin();       // Initialize sensors
    void readSensors(); // Read sensor data
};

#endif
