#include "esc_pwm.h"

float setDutyCycle(float dutyCycle) {
  float result;
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
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
    delay(20);
  }

  return 0;
}

// Changing direction function
// Example implementation: set_direction(FORWARDS);
int set_direction(bool direction) {
  unsigned long setDirectionMillis;
  if (direction == FORWARDS & DIRECTION_FLAG == BACKWARDS) {
    // set forwards for set length
    setDirectionMillis= millis();
    while(millis() - setDirectionMillis < RETURN_TO_NEUTRAL_LENGTH) {
      ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(70));
    }
    // set neutral for set length
    setDirectionMillis= millis();
    while(millis() - setDirectionMillis < RETURN_TO_NEUTRAL_LENGTH) {
      ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
    }
    // set forwards
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(70));
  }

  else if (direction == BACKWARDS & DIRECTION_FLAG == FORWARDS) {
    // set neutral for set length
    setDirectionMillis= millis();
    while(millis() - setDirectionMillis < RETURN_TO_NEUTRAL_LENGTH) {
      ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
    }
    // set forwards
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(79));
  }

  else if (direction == BACKWARDS & DIRECTION_FLAG == BACKWARDS) {
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(79));
  }
  
  else if (direction == FORWARDS & DIRECTION_FLAG == FORWARDS){
    ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(70));
  }

  DIRECTION_FLAG = direction;
  return 0;
}

int stop_motors() {
  ledcWrite(ESC_PWM_CHANNEL, setDutyCycle(75));
  return 0;
}