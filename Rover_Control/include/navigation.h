#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "config.h"
#include "ultrasonic.h"

int sweepFrontServo(int stepSize);
bool analyseForCan(int numberOfScans);

#endif  // CONFIG_H