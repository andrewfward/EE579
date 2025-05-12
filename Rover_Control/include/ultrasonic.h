#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "config.h"

int getUltrasoundValue(int trigPin);
void calculateInitialOffset(void);
void setOffsetBasedOnOneSide(bool);
void IRAM_ATTR echoL();
void IRAM_ATTR echoR();
void IRAM_ATTR echoF();

#endif  // CONFIG_H