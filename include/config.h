#ifndef CONFIG_H
#define CONFIG_H

// WiFi and mDNS
#define HOSTNAME    "DIT-SIMA-01"

// ESP-NOW for SIMA communication
#define BROADCAST_ADDR { 0x94, 0xa9, 0x90, 0x0b, 0x86, 0xd8 } // [94:a9:90:0b:86:d8]---[03]

// RGB LED strip 
#define LED_PIN             3
#define LED_COUNT           5
#define LED_BRIGHTNESS      128

#endif
