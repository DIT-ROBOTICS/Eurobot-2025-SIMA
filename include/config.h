#ifndef CONFIG_H
#define CONFIG_H

// WiFi and mDNS
#define HOSTNAME    "DIT-SIMA-00"

// ESP-NOW for SIMA communication
#define BROADCAST_ADDR { 0x94, 0xa9, 0x90, 0x0b, 0x86, 0xd8 } // [94:a9:90:0b:86:d8]---[03]

// RGB LED strip 
#define LED_PIN             3
#define LED_COUNT           20
#define LED_BRIGHTNESS      128

// VL53L0X sensors pins
#define VL53L0X_SDA_PIN     8
#define VL53L0X_SCL_PIN     9
#define XSHUT1_PIN          10
#define XSHUT2_PIN          11
#define XSHUT3_PIN          12

// Stepper motor pins
#define MS1_PIN             4
#define MS2_PIN             5
#define STEP_PIN_L          6
#define DIR_PIN_L           7
#define STEP_PIN_R          15
#define DIR_PIN_R           16

// Servo pins
#define servoPinR           19
#define servoPinL           20

#endif
