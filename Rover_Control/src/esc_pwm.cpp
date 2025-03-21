#include <Arduino.h>
#include "esc_pwm.h"

int motorStartupSequence() {
  unsigned long startMillis;

  // PWM settings
  static int ESC_PWM_PIN = 18;
  static int ESC_PWM_CHANNEL = 0;
  static int ESC_PWM_FREQ = 50;
  static int ESC_PWM_RESOLUTION = 8;

  // Configure PWM
  ledcAttachPin(ESC_PWM_PIN, ESC_PWM_CHANNEL);
  ledcSetup(ESC_PWM_CHANNEL, ESC_PWM_FREQ, ESC_PWM_RESOLUTION);

  // Set initial duty cycle to 7.5%
  ledcWrite(ESC_PWM_CHANNEL, 7.5 * 255 / 100);

  // Run startup sequence for a period defined in the header file
  startMillis = millis();
  while (millis() - startMillis < STARTUP_SEQUENCE_LENGTH) {
  // while(1){
    ledcWrite(ESC_PWM_CHANNEL, 7.5 * 255 / 100);
    delay(20);
  }

  return 0;
}
