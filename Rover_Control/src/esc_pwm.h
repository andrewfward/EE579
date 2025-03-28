#ifndef ESC_PWWM_H
#define ESC_PWWM_H

#include <Arduino.h>
#include "config.h"

#define STARTUP_SEQUENCE_LENGTH 3000 // milliseconds
#define RETURN_TO_NEUTRAL_LENGTH 250 // the length of time we return to neutral when switching from forward to reverse.
// PWM settings
static int ESC_PWM_PIN = 17;
static int ESC_PWM_CHANNEL = 0;
static int ESC_PWM_FREQ = 500;
static int ESC_PWM_RESOLUTION = 8;

float setDutyCycle(float);
int motorStartupSequence();
int set_direction(bool direction);
int stop_motors();

#define FORWARDS 1
#define BACKWARDS 0

#endif  // CONFIG_H