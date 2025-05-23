#ifndef CONFIG_H
#define CONFIG_H

// SIMA number              // TO-DO: Change SIMA number here
#define SIMA_NUM            0

// WiFi and mDNS            // TO-DO: Change hostname here
#define HOSTNAME            "DIT-SIMA-00"

// ESP-NOW for SIMA communication
//  [94:a9:90:07:00:78]---[01]
//  [94:a9:90:06:E6:00]---[02]
//  [94:a9:90:0b:86:d8]---[03]
//  [94:a9:90:05:57:d8]---[04]
//  [94:a9:90:0b:64:f0]---[05]
//  [94:a9:90:06:e6:b4]---[06]
//  [94:a9:90:05:4d:48]---[07]
//  [94:a9:90:05:57:f4]---[08]
//  --------------------------
//  [94:a9:90:0b:bb:bc]---[11] (07)

// SIMA MAC Addresses
#define SIMA_01 { 0x94, 0xa9, 0x90, 0x07, 0x00, 0x78 }
#define SIMA_02 { 0x94, 0xa9, 0x90, 0x06, 0xe6, 0x00 }
#define SIMA_03 { 0x94, 0xa9, 0x90, 0x0b, 0x86, 0xd8 }
#define SIMA_04 { 0x94, 0xa9, 0x90, 0x05, 0x57, 0xd8 }
#define SIMA_05 { 0x94, 0xa9, 0x90, 0x0b, 0x64, 0xf0 }
#define SIMA_06 { 0x94, 0xa9, 0x90, 0x06, 0xe6, 0xb4 }
#define SIMA_07 { 0x94, 0xa9, 0x90, 0x05, 0x4d, 0x48 }
#define SIMA_08 { 0x94, 0xa9, 0x90, 0x05, 0x57, 0xf4 }

#define SIMA_09 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TO-DO: Replace with actual MAC address
#define SIMA_10 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TO-DO: Replace with actual MAC address
#define SIMA_11 { 0x94, 0xa9, 0x90, 0x0b, 0xbb, 0xbc }
#define SIMA_12 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TO-DO: Replace with actual MAC address

// Voltmeter - Battery voltage measurement
// | Formula:
// |    Vbattf = (VOLTMETER_CALIBRATION * Vbatt / SLIDING_WINDOW_SIZE / 1000.0) + VOLTMETER_OFFSET;
// |    [ R1 = 22k ohm, R2 = 8.2k ohm ] VC = 3.68 OFFSET = 0.00
// |    [ R1 = 33k ohm, R2 = 10k ohm ]  VC = 4.30 OFFSET = 0.00   // RECOMMENDED
// |
#define VOLTMETER_PIN           14      // Second-to-last pin
#define VOLTMETER_CALIBRATION   4.3
#define VOLTMETER_OFFSET        0.25
#define SLIDING_WINDOW_SIZE     64
#define TIMER_PERIOD_US         1000000

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
#define STEP_BIT_L (1 << STEP_PIN_L)
#define DIR_PIN_L           7
#define STEP_PIN_R          15
#define STEP_BIT_R (1 << STEP_PIN_R)
#define DIR_PIN_R           16

// Servo pins
#define servoPinR           19
#define servoPinL           20


#endif
