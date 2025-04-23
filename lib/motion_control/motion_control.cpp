#include "motion_control.h"
#include "config.h"
#include "sima_core.h"
#include <math.h>
#include <Arduino.h>

void goForward(float distance) {
    switchcase();
    going = true;
    stepDelayL = maxStepDelay;
    stepDelayR = maxStepDelay;
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL = distance;
    distanceR = distance;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    accelerationL = true;
    accelerationR = true;
}

void goBackward(float distance) {
    switchcase();
    goingBack = true;
    stepDelayL = maxStepDelay;
    stepDelayR = maxStepDelay;
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, LOW);
    distanceL = distance;
    distanceR = distance;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
    accelerationL = true;
    accelerationR = true;
}

void turnLeft(float degree) {
    switchcase();
    rotatingL = true;
    stepDelayL = 60;
    stepDelayR = 60;
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, HIGH);
    distanceL = carCircum * degree / 360;
    distanceR = carCircum * degree / 360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
}

void turnRight(float degree) {
    switchcase();
    rotatingR = true;
    stepDelayL = 60;
    stepDelayR = 60;
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, LOW);
    distanceL = carCircum * degree / 360;
    distanceR = carCircum * degree / 360;
    esp_timer_start_once(stepperTimerL, stepDelayL);
    esp_timer_start_once(stepperTimerR, stepDelayR);
}

void goToTheta(float x_2, float y_2) {
    float dx = x_2 - x_1;
    float dy = y_2 - y_1;
    float thetaGoal = atan2f(dy, dx) * 180 / M_PI;
    float thetaDiff = thetaGoal - theta;
    if (thetaDiff > 180)    thetaDiff -= 360;
    if (thetaDiff < -180)   thetaDiff += 360;
    if (thetaDiff > 0)      turnLeft ( thetaDiff);
    else                    turnRight(-thetaDiff);
}

void goToDistance(float x_2, float y_2) {
    float dx = x_2 - x_1;
    float dy = y_2 - y_1;
    float distance = sqrt(pow(dx, 2) + pow(dy, 2));
    goForward(distance);
}