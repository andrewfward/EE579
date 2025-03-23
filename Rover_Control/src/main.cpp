
#include "pwm.h"
#include "config.h"
#include <cmath> // For abs() in C++


void calculateInitialOffset();

void IRAM_ATTR echoL() { 
  if (digitalRead(echoPinL)) {
    startTimeL = micros();  // Echo started
  } else {
    endTimeL = micros();  // Echo ended
    receivedL = true;
  }
}

void IRAM_ATTR echoR() { 
  if (digitalRead(echoPinR)) {
    startTimeR = micros();  // Echo started
  } else {
    endTimeR = micros();  // Echo ended
    receivedR = true;
  }
}


void ultrasoundTask(void *pvParameters) {
  float prevPos = 0;
  float newPos = 0;
  int distanceL = 0;
  int distanceR = 0;
  int prevDistanceR = 0;
  int prevDistanceL = 0;

  // control param 
  float Kp = 1.0;
  float Ki = 0.0;
  float error = 0.0;
  float eIntegral = 0.0;
  float timeStep = 40.0/1.0e3;
  for (;;) {
    
    digitalWrite(trigPinL, LOW);
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    // send trigger pulse for 10 us 
    digitalWrite(trigPinL, HIGH);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    digitalWrite(trigPinR, LOW);

    vTaskDelay(pdMS_TO_TICKS(20)); // wait max time for signals to be recived

    if (receivedL) {
      distanceL = (endTimeL - startTimeL)/58;
      if (distanceL > 400) {
        distanceL = 400;
      }
      SerialBT.print("Left Ultrasonic Sensor: ");
      SerialBT.println(distanceL);
      receivedL = false;
    }

    if (receivedR) {
      distanceR = (endTimeR - startTimeR)/58;
      if (distanceR > 400) {
        distanceR = 400;
      }
      SerialBT.print("Right Ultrasonic Sensor: ");
      SerialBT.println(distanceR);
      receivedR = false;
    }
    
    newPos = distanceR - distanceL - initialOffset;

    // only update pos when a chnage greater than 1 occurs
    // this is to reduce the effct of noise
    // prePos is only updated after the chnage is greater than 1
    // so a slow chnage in pos will still have an effect after multiple iterations
    if (fabs(newPos - prevPos) > 1) {
      pos = (float)newPos;
      prevPos = newPos;
    }

    prevDistanceR = distanceR;
    prevDistanceL = distanceL;

    SerialBT.print("Lateral Position: ");
    SerialBT.println(pos);

    // add function to detect landmarks (ie dips in the corridor)
    if (abs(distanceR - prevDistanceR) > 20) {
      landmarkCounter += 1;
      landmarkFlag = true;
    }

    // control code to convert that pos to an angle (1100 - 1800)
    // where 1500 is 0
    // PI controller

    error = 0.0 - pos;
    eIntegral = eIntegral + error*timeStep;
    steeringAngle = (int)(neutralPos + (Kp * error) + (Ki * eIntegral));

    SerialBT.print("Steering Angle (us): ");
    SerialBT.println(steeringAngle);
    SerialBT.println("");

    // saturation constarints
    if (steeringAngle > maxUs) {
      steeringAngle = maxUs;
    }

    if (steeringAngle < minUs) {
      steeringAngle = minUs;
    }

    servoSteering.writeMicroseconds(steeringAngle);

    vTaskDelay(pdMS_TO_TICKS(20)); // wait max time for signals to be recived
  }
}

void moveToAreaTask(void *pvParameters) {
    float estimatedDistance = 0.0;
    float targetDistance = 0.0;

    long startTimeDistance = millis();
    estimatedDistance = 0.0;

    // a maximum drive time in case the landmarks dont work
    long maxTime = 10000;

    // code to set the speed of the motor 
    // probabily using ledcwrite
    // 50 Hz (5-10 % duty cycle with 7.5 % being neutral

    float estimatedSpeed = 0.1;   // guess based on testing (currently random)

    while (estimatedDistance < targetDistance || millis() - startTimeDistance < maxTime) {

      estimatedDistance += estimatedSpeed * 0.1;

      if (landmarkFlag) {
        estimatedDistance = landmarkDistances[landmarkCounter];
        landmarkFlag = false;
      }
      
      vTaskDelay(pdMS_TO_TICKS(100)); // Delay AFTER execution

    }

    // stop motors 
    
    // pause or delete task, havent decided yet if it wil be reused on the return path
    vTaskSuspend(moveToAreaTaskHandle);
  
}

void setup() {
    //Serial.begin(115200); // Start Serial Monitor

    ESP32PWM::allocateTimer(2);  // Allocate one timer for the steering 
    ESP32PWM::allocateTimer(3);  // Allocate one timer for the ultraounic sensor servo 

    servoSteering.setPeriodHertz(300);    // set frequency of PWM signal to 300 Hz
    servoSteering.attach(steeringServoPin, minUs, maxUs);

    pinMode(trigPinL, OUTPUT);
    pinMode(echoPinL, INPUT);

    pinMode(trigPinR, OUTPUT);
    pinMode(echoPinR, INPUT);

    // Attach interrupts to handle echo signals
    attachInterrupt(digitalPinToInterrupt(echoPinL), echoL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(echoPinR), echoR, CHANGE);

    SerialBT.begin("ESP32_BT_MC"); // Set Bluetooth device name
    delay(5000);

    calculateInitialOffset();
    delay(1000);

    // Create the side ultrasound sensor task
    xTaskCreate(
      ultrasoundTask,            // Task function
      "Side Ultrasound",         // Task name
      2048,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &ultrasoundTaskHandle      // Task handle
    );

    // Create the side ultrasound sensor task
    xTaskCreate(
      moveToAreaTask,            // Task function
      "move to task area",         // Task name
      2048,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &moveToAreaTaskHandle      // Task handle
    );

    // suspend for the minute until the other sections have been tested
    vTaskSuspend(moveToAreaTaskHandle);
    delay(1000);
    
    

    // code for ESC setup routine
    //motorStartupSequence();    
}

void loop() {

}

void calculateInitialOffset() {
  int distanceL = 0;
  int distanceR = 0;
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  // send trigger pulse for 10 us 
  digitalWrite(trigPinL, HIGH);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);

  delay(20);

  if (receivedL) {
    distanceL = (endTimeL - startTimeL)/58;
    //SerialBT.print("Left Ultrasonic Sensor: ");
    //SerialBT.println(distanceL);
    receivedL = false;
  }

  if (receivedR) {
    distanceR = (endTimeR - startTimeR)/58;
    //SerialBT.print("Left Ultrasonic Sensor: ");
    //SerialBT.println(distanceR);
    receivedR = false;
  }

  initialOffset = distanceR - distanceL;

  SerialBT.print("Initial Offset (cm): ");
  SerialBT.println(initialOffset);
}