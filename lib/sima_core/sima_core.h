#pragma once

#include <Arduino.h>
#include <esp_timer.h>
#include "config.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"   


#define maxStepDelay 180
#define minStepDelay 55
#define accRate 0.015
#define decRate 0.015
#define decDistance 200
#define ACC_FIXED 33000
#define SCALE     1000000000ULL


// Declare global variables as extern
extern float stepDelayL, stepDelayR, preStepDelayL, preStepDelayR;
extern bool accelerationL, accelerationR, decelerationL, decelerationR;
extern bool rotatingL, rotatingR, going, goingBack, reach_goal;
extern bool avoiding;
extern float x_1, y_1, theta, x_goal, y_goal;
extern float distanceL, distanceR, range;
extern float mission, avoidStage, firstSimaStepStage, escape, adjust;
extern int step, preStep, test;
extern esp_timer_handle_t stepperTimerL, stepperTimerR, goalCheckTimer;

// Declare start_reach_goal as extern to link it across files
extern volatile bool start_reach_goal;

void sima_core(void *parameter);
void sima_core_1(void *parameter);
void sima_core_2(void *parameter);
void sima_core_superstar(void *parameter);
void initSimaCore();
void firstSimaStep(int num);
void switchcase();
void setSimaGoal(int num);