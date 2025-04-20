#pragma once

#include <Arduino.h>
#include <esp_timer.h>

// Declare global variables as extern
extern float stepDelayL, stepDelayR;
extern bool accelerationL, accelerationR, decelerationL, decelerationR;
extern bool rotatingL, rotatingR, going, goingBack, reach_goal;
extern bool avoiding;
extern float x_1, y_1, theta, x_goal, y_goal;
extern float distanceL, distanceR, range;
extern float missionL, missionR, avoidStageL, avoidStageR, escape, adjust;
extern int step, preStep, test;
extern esp_timer_handle_t stepperTimerL, stepperTimerR, goalCheckTimer;

#define servoPinR 19 //R:servo 1 
#define servoPinL 20 //L:servo 2  

#define MS1_PIN 4   
#define MS2_PIN 5
// Declare start_reach_goal as extern to link it across files

extern volatile bool start_reach_goal;

void sima_core(void *parameter);
void initSimaCore();