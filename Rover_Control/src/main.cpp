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

    // Send the updated data over Bluetooth serial
    String logEntry = String(distanceL) + "," + String(distanceR) + "," + String(steeringAngle);
    SerialBT.println(logEntry);

    vTaskDelay(pdMS_TO_TICKS(20));

    if (!RUN) {
      vTaskSuspend(NULL);
    }
  }
}

void moveToAreaTask(void *pvParameters) {
  set_direction(FORWARDS);
  long maxTime = 10000; // 10 seconds max
  long startTimeDistance = millis();

  while (RUN && (millis() - startTimeDistance < maxTime)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  moving = false;
  logDataReady = true;
  RUN = false;
  vTaskSuspend(moveToAreaTaskHandle);
}

// Bluetooth communication task
void bluetoothTask(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim();

      if (command == "start") {
        RUN = true;
        moving = true;
        logDataReady = false;
        vTaskResume(ultrasoundTaskHandle);
        vTaskResume(moveToAreaTaskHandle);
        SerialBT.println("Start Command Received");
      } 
      else if (command == "stop") {
        RUN = false;
        stop_motors();
        moving = false;
        logDataReady = true;
        SerialBT.println("Stop Command Received");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
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

  calculateInitialOffset();
  delay(1000);
  
  motorStartupSequence();
  delay(1000);

  // Create tasks
  xTaskCreatePinnedToCore(ultrasoundTask, "Side Ultrasound", 4000, NULL, 1, &ultrasoundTaskHandle, 1);
  vTaskSuspend(ultrasoundTaskHandle);
  xTaskCreatePinnedToCore(moveToAreaTask, "Move to Area", 4000, NULL, 1, &moveToAreaTaskHandle, 1);
  vTaskSuspend(moveToAreaTaskHandle);
  xTaskCreatePinnedToCore(bluetoothTask, "Bluetooth Comms", 4000, NULL, 1, NULL, 1);
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
