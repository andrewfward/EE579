#include "ultrasonic.h"

// returns the distance from the called US sensor
int getUltrasoundValue(int trigPin) {
    int distance = -1;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    if (trigPinF) {
      vTaskDelay(pdMS_TO_TICKS(90));
    } else {
      vTaskDelay(pdMS_TO_TICKS(55));
    }

    if (trigPin == trigPinL) {
        if (receivedL) {
            distance = (endTimeL - startTimeL) / 58;
            if (distance > 450) {
              distance = 450;
            }
            receivedL = false;
        }
    } else if (trigPin == trigPinR) {
        if (receivedR) {
            distance = (endTimeR - startTimeR) / 58;
            if (distance > 450) {
              distance = 450;
            }
            receivedR = false;
        }
    } else if (trigPin == trigPinF) {
        if (receivedF) {
            distance = (endTimeF - startTimeF) / 58;
            if (abs(distance) > 450) {
              distance = 450;
            }
            receivedF = false;
        }
    } else {
        SerialBT.println("CAN: Invalid Sensor");
    }
    return distance;
}

void calculateInitialOffset(void) {
    int distanceL = 0;
    int distanceR = 0;
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
  
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    
    delay(60);
  
    if (receivedL) {
      initialOffsetL = (endTimeL - startTimeL) / 58;
      receivedL = false;
    }
  
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
    delay(60);
  
    if (receivedR) {
      initialOffsetR = (endTimeR - startTimeR) / 58;
      receivedR = false;
    }
  }

  void setOffsetBasedOnOneSide(bool side) {
    if (side == LEFT){
      SerialBT.println("CAN:Calculating based on LHS...");
      digitalWrite(trigPinL, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinL, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinL, LOW);
      
      delay(60);
  
      if (receivedL) {
        initialOffsetL = (endTimeL - startTimeL) / 58;
        initialOffsetR = 300 - initialOffsetL; //estimating the width of the corridor minus the width of the car 
        receivedL = false;
      }
    } 
    else {
      SerialBT.println("CAN:Calculating based on RHS...");
      digitalWrite(trigPinR, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinR, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinR, LOW);
      
      delay(60);
  
      if (receivedR) {
        initialOffsetR = (endTimeR - startTimeR) / 58;
        initialOffsetL = 300 - initialOffsetR; //estimating the width of the corridor minus the width of the car 
        receivedR = false;
      }
    }
  }


  // Interrupts for each ultrasound sensor
void IRAM_ATTR echoL() { 
  if (digitalRead(echoPinL)) {
    startTimeL = micros();
  } else {
    endTimeL = micros();
    receivedL = true;
  }
}

void IRAM_ATTR echoR() { 
  if (digitalRead(echoPinR)) {
    startTimeR = micros();
  } else {
    endTimeR = micros();
    receivedR = true;
  }
}

void IRAM_ATTR echoF() { 
  if (digitalRead(echoPinF)) {
    startTimeF = micros();
  } else {
    endTimeF = micros();
    receivedF = true;
  }
}