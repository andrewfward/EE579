#include "esc_pwm.h"

float setDutyCycle(float dutyCycle) {
  result = dutyCycle * 255 / 100;
  return result;
}

int motorStartupSequence() {
  unsigned long startMillis;

  // Configure PWM
  ledcAttachPin(ESC_PWM_PIN, ESC_PWM_CHANNEL);
  ledcSetup(ESC_PWM_CHANNEL, ESC_PWM_FREQ, ESC_PWM_RESOLUTION);

  // Set initial duty cycle to 75%
  ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));

  // Run startup sequence for a period defined in the header file
  startMillis = millis();
  while (millis() - startMillis < STARTUP_SEQUENCE_LENGTH) {
  // while(1){
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
    delay(20);
  }

  return 0;
}

// Changing direction function
// Example implementation: set_direction(FORWARDS);
int set_direction(bool direction) {
  unsigned long returnToNeutralMillis;
  // return to neutral for one second
  returnToNeutralMillis = millis();
  while(millis() - returnToNeutralMillis < RETURN_TO_NEUTRAL_LENGTH) {
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
  }
  
  if (direction==FORWARDS) {
    ledcwrite(ESC_PWM_CHANNEL, setDutyCycle(73));
  }
  else{
    ledcwrite(ESC_PWM_CHANNEL, setDutyCycle(77));
  }

  return 0;
}