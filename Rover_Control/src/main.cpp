#include "esc_pwm.h"
#include "config.h"
#include <cmath> // For abs() in C++

// Function declarations
void calculateInitialOffset();

// Interrupt handlers
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

// Tasks
void ultrasoundTask(void *pvParameters) {
  float prevPos = 0;
  float newPos = 0;
  int distanceL = 0;
  int distanceR = 0;
  int prevDistanceR = 0;
  int prevDistanceL = 0;

  float Kp = 1;
  float Ki = 0.1;
  float error = 0.0;
  float eIntegral = 0.0;
  float timeStep = 60.0/1.0e3;

  for (;;) {
    digitalWrite(trigPinL, LOW);
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPinL, HIGH);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    digitalWrite(trigPinR, LOW);

    vTaskDelay(pdMS_TO_TICKS(30));

    if (receivedL) {
      distanceL = (endTimeL - startTimeL) / 58;
      if (distanceL > 400) distanceL = 400;
      receivedL = false;
    }

    if (receivedR) {
      distanceR = (endTimeR - startTimeR) / 58;
      if (distanceR > 400) distanceR = 400;
      receivedR = false;
    }

    posR = distanceR - initialOffsetR;
    posL = distanceL - initialOffsetL;

    if (posL < 1.0 && posR > 1.0 || posL > 1.0 && posR < 1.0) {
      pos = posL;
    }
    else if (abs(posL) < 1.0 && (posR > -1.0 && posR < 1.0)) {
      pos = -posR;
    }
    else if (abs(posR) < 1.0 && (posL > -1.0 && posL < 1.0)) {
      pos = posL;
    } 
    else {
      pos = 0;
    }

    error = 0.0 - pos;
    eIntegral += error * timeStep;
    steeringAngle = (int)(neutralPos + (Kp * error) + (Ki * eIntegral));

    if (steeringAngle > maxUs) steeringAngle = maxUs;
    if (steeringAngle < minUs) steeringAngle = minUs;

    servoSteering.writeMicroseconds(steeringAngle);

    // Save to log
    if (moving && logIndex < MAX_LOG_ENTRIES) {
      logData[logIndex].distanceL = distanceL;
      logData[logIndex].distanceR = distanceR;
      logData[logIndex].steering = steeringAngle;
      logIndex++;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void moveToAreaTask(void *pvParameters) {
  float estimatedDistance = 0.0;
  float targetDistance = 0.0;
  long maxTime = 10000; // 10 seconds max
  float estimatedSpeed = 0.01;
  long startTimeDistance = millis();
  
  set_direction(FORWARDS);

  while (moveEnabled && (millis() - startTimeDistance < maxTime)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  moving = false;
  logDataReady = true;
  vTaskSuspend(moveToAreaTaskHandle);
}


// bluetooth coms task runs every 100 ms
void bluetoothTask(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim();

      if (command == "start") {
        moveEnabled = true;
        if (!moving) {
          logIndex = 0;
          vTaskResume(moveToAreaTaskHandle);
          moving = true;
        }
      } else if (command == "stop") {
        moveEnabled = false;
        stop_motors();
        moving = false;
        logDataReady = true;
      }
    }

    if (logDataReady) {
      // Output header
      SerialBT.println("DistanceLeft,DistanceRight,SteeringAngle");
      for (int i = 0; i < logIndex; i++) {
        String logEntry = String(logData[i].distanceL) + "," +
                          String(logData[i].distanceR) + "," +
                          String(logData[i].steering);
        SerialBT.println(logEntry);
      }
      logDataReady = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  //Serial.begin(115200); // Optional for Serial Monitor

  SerialBT.begin("ESP32_Rover");

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoSteering.setPeriodHertz(300);
  servoSteering.attach(steeringServoPin, minUs, maxUs);

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  attachInterrupt(digitalPinToInterrupt(echoPinL), echoL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinR), echoR, CHANGE);
  delay(1000);

  calculateInitialOffset();
  delay(1000);

  motorStartupSequence();
  delay(1000);

  // Create ultrasound task
  xTaskCreatePinnedToCore(
    ultrasoundTask,
    "Side Ultrasound",
    4000,
    NULL,
    1,
    &ultrasoundTaskHandle,
    1
  );

  // Create move task
  xTaskCreatePinnedToCore(
    moveToAreaTask,
    "Move to Area",
    4000,
    NULL,
    1,
    &moveToAreaTaskHandle,
    1
  );

  vTaskSuspend(moveToAreaTaskHandle);

  // Create Bluetooth task
  xTaskCreatePinnedToCore(
    bluetoothTask,
    "Bluetooth Comms",
    4000,
    NULL,
    1,
    NULL,
    1
  );

  delay(1000);
}

void loop() {
  // Nothing needed here
}

void calculateInitialOffset() {
  int distanceL = 0;
  int distanceR = 0;
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPinL, HIGH);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);

  delay(30);

  if (receivedL) {
    initialOffsetL = (endTimeL - startTimeL) / 58;
    receivedL = false;
  }

  if (receivedR) {
    initialOffsetR = (endTimeR - startTimeR) / 58;
    receivedR = false;
  }
}
