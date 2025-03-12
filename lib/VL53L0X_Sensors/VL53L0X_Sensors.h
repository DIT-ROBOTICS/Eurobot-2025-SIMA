#ifndef VL53L0X_SENSORS_H
#define VL53L0X_SENSORS_H

#include <Wire.h>
#include <VL53L0X.h>

extern float VL53L, VL53M, VL53R;

class VL53L0X_Sensors {
public:
    VL53L0X sensor1, sensor2, sensor3;
    
    // XSHUT GPIO 腳位
    const int XSHUT1 = 10;
    const int XSHUT2 = 11;
    const int XSHUT3 = 12;

    // I2C 腳位
    const int SDA_PIN = 8;
    const int SCL_PIN = 9;

    VL53L0X_Sensors();  // 建構函式
    void begin();       // 初始化感測器
    void readSensors(); // 讀取感測器數據
};

#endif
