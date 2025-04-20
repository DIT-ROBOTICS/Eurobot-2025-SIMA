#pragma once

#include <Arduino.h>
#include "config.h"

#define wheelCircum         47.17 * PI              // mm
#define lengthPerStep       wheelCircum / 12800
#define wheelDistance       85.53                   // mm (by tiral)
#define carCircum           wheelDistance * PI
#define STEPS_PER_REV       12800  // 200 * 32 = 6400 microsteps, 6400 * 2
#define rotateAnglePerStep  0.0155

void goForward(float distance);
void goBackward(float distance);
void turnLeft(float degree);
void turnRight(float degree);
void goToTheta(float x_2, float y_2);
void goToDistance(float x_2, float y_2);