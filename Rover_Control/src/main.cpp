
#include "esc_pwm.h"
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
  float Kp = 1;
  float Ki = 0.1;
  float error = 0.0;
  float eIntegral = 0.0;
  float timeStep = 60.0/1.0e3;
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

    vTaskDelay(pdMS_TO_TICKS(30)); // wait max time for signals to be recived

    if (receivedL) {
      distanceL = (endTimeL - startTimeL)/58;
      if (distanceL > 400) {
        distanceL = 400;
      }
      //SerialBT.print("Left Ultrasonic Sensor: ");
      //SerialBT.println(distanceL);
      receivedL = false;
    }

    if (receivedR) {
      distanceR = (endTimeR - startTimeR)/58;
      if (distanceR > 400) {
        distanceR = 400;
      }
      //SerialBT.print("Right Ultrasonic Sensor: ");
      //erialBT.println(distanceR);
      receivedR = false;
    }
    
    posR = distanceR - initialOffsetR;
    posL = distanceL - initialOffsetL;
      // car move to the left or right 
    if (posL < 1.0 && posR > 1.0 || posL > 1.0 && posR < 1.0) {
      pos = posL;
    }
    // wall chnaged on the left but not the right
    else if (abs(posL) < 1.0 && (posR > -1.0 && posR < 1.0)) {
      pos = -posR;
    }
    // wall chnaged on the right but not the left
    else if (abs(posR) < 1.0 && (posL > -1.0 && posL < 1.0)) {
      pos = posL;
    } else {
      pos = 0;
    }

          /*
      if (abs(distanceL - prevDistanceL) > 5 || abs(distanceR - prevDistanceR) > 5) {
        if (abs(distanceL - prevDistanceL) > 5) {
          initialOffsetL = distanceL;
        }
        if (abs(distanceR - prevDistanceR) > 5) {
          initialOffsetR = distanceR;
        }
        posL = distanceL - initialOffsetL;
        pos = posL;
      }
        */
  
    // only update pos when a chnage greater than 1 occurs
    // this is to reduce the effct of noise
    // prePos is only updated after the chnage is greater than 1
    // so a slow chnage in pos will still have an effect after multiple iterations
    /*
    if (fabs(newPos - prevPos) > 1) {
      pos = (float)newPos;
      prevPos = newPos;
    }
      */

    prevDistanceR = distanceR;
    prevDistanceL = distanceL;

    //SerialBT.print("Lateral Position: ");
    //SerialBT.println(pos);
    // control code to convert that pos to an angle (1100 - 1800)
    // where 1500 is 0
    // PI controller

    error = 0.0 - pos;
    eIntegral = eIntegral + error*timeStep;
    steeringAngle = (int)(neutralPos + (Kp * error) + (Ki * eIntegral));

    //SerialBT.print("Steering Angle (us): ");
    //SerialBT.println(steeringAngle);
    //SerialBT.println("");

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

    
    estimatedDistance = 0.0;

    // a maximum drive time in case the landmarks dont work
    long maxTime = 10000;

    // code to set the speed of the motor 
    // probabily using ledcwrite
    // 50 Hz (5-10 % duty cycle with 7.5 % being neutral
    
    //SerialBT.println("started");

    float estimatedSpeed = 0.01;   // guess based on testing (currently random)
    long startTimeDistance = millis();
    set_direction(FORWARDS);

    while (millis() - startTimeDistance < maxTime) {

      estimatedDistance += estimatedSpeed * 0.1;
      
      vTaskDelay(pdMS_TO_TICKS(10)); // Delay AFTER execution
    }

    // stop motors 
    stop_motors();
    //SerialBT.println("stopped");
    
    // pause or delete task, havent decided yet if it wil be reused on the return path
    vTaskSuspend(moveToAreaTaskHandle);
}

void setup() {

    //Serial.begin(115200); // Start Serial Monitor

    ESP32PWM::allocateTimer(2);  // Allocate one timer for the steering 
    ESP32PWM::allocateTimer(3);  // Allocate one timer for the ultrasonic sensor servo 

    servoSteering.setPeriodHertz(300);    // set frequency of PWM signal to 300 Hz
    servoSteering.attach(steeringServoPin, minUs, maxUs);

    pinMode(trigPinL, OUTPUT);
    pinMode(echoPinL, INPUT);

    pinMode(trigPinR, OUTPUT);
    pinMode(echoPinR, INPUT);

    // Attach interrupts to handle echo signals
    attachInterrupt(digitalPinToInterrupt(echoPinL), echoL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(echoPinR), echoR, CHANGE);
    delay(1000);

    //SerialBT.begin("ESP32_BT"); // Set Bluetooth device name
    delay(5000);

    calculateInitialOffset();
    delay(1000);

    // ESC startup routine
    motorStartupSequence();   
    delay(1000);

    // Create the side ultrasound sensor task on core 1
    xTaskCreatePinnedToCore(
      ultrasoundTask,            // Task function
      "Side Ultrasound",         // Task name
      4000,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority
      &ultrasoundTaskHandle,     // Task handle
      1                          // Core to pin the task to (0 or 1)
    );

    // Create the moveToArea task on core 1
    xTaskCreatePinnedToCore(
      moveToAreaTask,            // Task function
      "Move to Area",            // Task name
      4000,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority
      &moveToAreaTaskHandle,     // Task handle
      1                          // Core to pin the task to (same as above)
    );

    delay(1000);
   
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

  delay(30);

  if (receivedL) {
    initialOffsetL = (endTimeL - startTimeL)/58;
    //SerialBT.print("Left Ultrasonic Sensor: ");
    //SerialBT.println(distanceL);
    receivedL = false;
  }

  if (receivedR) {
    initialOffsetR = (endTimeR - startTimeR)/58;
    //SerialBT.print("Left Ultrasonic Sensor: ");
    //SerialBT.println(distanceR);
    receivedR = false;
  }

  //SerialBT.print("Initial Offset R (cm): ");
  //SerialBT.println(initialOffsetR);

  //SerialBT.print("Initial Offset L (cm): ");
  //SerialBT.println(initialOffsetL);
}