#include "voltmeter.h"
#include "config.h"

#include "led_control.h"
#include "web_interface.h"

float Vbattf = 0.0;
uint32_t Vbatt = 0;
float offset = VOLTMETER_OFFSET;
volatile int interruptCounter = 0;
hw_timer_t* _timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
  }

void initVoltmeter() {
  pinMode(VOLTMETER_PIN, INPUT);

  _timer = timerBegin(10000);

  timerAttachInterrupt(_timer, &onTimer);
  timerStart(_timer);
}

void voltmeter() {
  Vbatt = 0;
  for (int i = 0; i < SLIDING_WINDOW_SIZE; i++) {
    Vbatt += analogReadMilliVolts(VOLTMETER_PIN);
  }
  Vbattf = VOLTMETER_CALIBRATION * Vbatt / SLIDING_WINDOW_SIZE / 1000.0 + offset;
  if (Vbattf < 3) Vbattf = 0.0;

  if (Vbattf < 9) { mode = LOW_BATTERY;  }
  else              { mode = DEFAULT_MODE; }
  // WebSerial.printf("Battery Voltage: %.2f V\n", Vbattf);
}

void voltmeterTask(void* pvParameters) {
  for (;;) {
    voltmeter();

    if (interruptCounter > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter--;
      portEXIT_CRITICAL(&timerMux);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}