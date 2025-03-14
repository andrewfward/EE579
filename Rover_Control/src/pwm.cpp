#include <Arduino.h>
#include "pwm.h"

int motorStartupSequence() {
  unsigned long startMillis;

  // PWM settings
  static int PWM_PIN = 18;
  static int PWM_CHANNEL = 0;
  static int PWM_FREQ = 50;
  static int PWM_RESOLUTION = 8;

  // Configure PWM
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // Set initial duty cycle to 7.5%
  ledcWrite(PWM_CHANNEL, 7.5 * 255 / 100);

  // Run startup sequence for a period defined in the header file
  startMillis = millis();
  while (millis() - startMillis < STARTUP_SEQUENCE_LENGTH) {
  // while(1){
    ledcWrite(PWM_CHANNEL, 7.5 * 255 / 100);
    delay(20);
  }

  return 0;
}
