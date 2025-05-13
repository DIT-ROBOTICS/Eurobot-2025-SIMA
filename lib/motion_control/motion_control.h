#pragma once

#include <Arduino.h>
#include "config.h"

#define STEPS_PER_REV       3200  // 200 * 8 = 1600 microsteps, 1600 * 2
#define wheelCircum         47.17 * PI              // mm
#define lengthPerStep       wheelCircum / STEPS_PER_REV // mm
#define wheelDistance       83                // mm (by tiral)
#define carCircum           wheelDistance * PI
#define rotateAnglePerStep  ((lengthPerStep) / (carCircum)) * 360


void goForward(float distance);
void goBackward(float distance);
void turnLeft(float degree);
void turnRight(float degree);
void goToTheta(float x_2, float y_2);
void goToDistance(float x_2, float y_2);