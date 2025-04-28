#ifndef CONFIG_H
#define CONFIG_H

// SIMA number              // TO-DO: Change SIMA number here
#define SIMA_NUM            1

// WiFi and mDNS            // TO-DO: Change hostname here
#define HOSTNAME            "DIT-SIMA-01"

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
#define DIR_PIN_L           7
#define STEP_PIN_R          15
#define DIR_PIN_R           16

// Servo pins
#define servoPinR           19
#define servoPinL           20


#endif
