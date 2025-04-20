#pragma once

#include <Arduino.h>

#define STEP_PIN_L 6   
#define DIR_PIN_L 7    
#define STEP_PIN_R 15   
#define DIR_PIN_R 16    

#define wheelCircum  47.17*PI//mm
#define lengthPerStep wheelCircum/12800//mm
#define wheelDistance 85.53// mm (by tiral)
#define carCircum wheelDistance*PI//(by tiral)
#define STEPS_PER_REV 12800  // 200 步 * 32 微步 = 6400 microsteps, 6400*2
#define rotateAnglePerStep  0.0155

void goForward(float distance);
void goBackward(float distance);
void turnLeft(float degree);
void turnRight(float degree);
void goToTheta(float x_2, float y_2);
void goToDistance(float x_2, float y_2);