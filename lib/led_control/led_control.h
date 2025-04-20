#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Adafruit_NeoPixel.h>

enum led_mode {
  DEFAULT_MODE,
  STATIC_COLOR,
  COLOR_WIPE,
  BREATHING,
  RAINBOW,
  LOW_BATTERY,
  UPDATING,
  WIFI_WIZARD,
  WIFI_DISCONNECTED,
};

extern Adafruit_NeoPixel strip;
extern volatile int mode;
extern volatile int sensor_mode;
extern unsigned long last_override_time;
extern const unsigned long override_duration;

void initLED();
void colorWipe(uint32_t color, int wait);
void rainbow(int wait);
void LEDTask(void* pvParameters);

#endif
