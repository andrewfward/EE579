#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "config.h"

int getUltrasoundValue(int trigPin);
void calculateInitialOffset(void);
void setOffsetBasedOnOneSide(bool);

#endif  // CONFIG_H