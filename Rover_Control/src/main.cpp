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

void IRAM_ATTR echoF() { 
  if (digitalRead(echoPinF)) {
    startTimeF = micros();
  } else {
    endTimeF = micros();
    receivedF = true;
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

  float Kp = 0.2;
  float Ki = 0.05;
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

    vTaskDelay(pdMS_TO_TICKS(55));

    if (receivedL) {
      distanceL = (endTimeL - startTimeL) / 58;
      if (distanceL > 450) {
        distanceL = 450;
      }
      receivedL = false;
    }

    if (receivedR) {
      distanceR = (endTimeR - startTimeR) / 58;
      if (distanceR > 450) {distanceR = 450;
      }
      receivedR = false;
    }

    posR = distanceR - initialOffsetR;
    posL = distanceL - initialOffsetL;

    pos = posR - posL;

    error = 0.0 - pos;
    eIntegral += error * timeStep;
    steeringAngle = (int)(neutralPos + (Kp * error) + (Ki * eIntegral));

    if (steeringAngle > maxUsSteer) steeringAngle = maxUsSteer;
    if (steeringAngle < minUsSteer) steeringAngle = minUsSteer;

    servoSteering.writeMicroseconds(steeringAngle);

    String logEntry = String(distanceL) + "," + String(distanceR) + "," + String(steeringAngle) + "," + String(pos);
    SerialBT.println(logEntry);

    if (!RUN) {
      vTaskSuspend(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void moveToAreaTask(void *pvParameters) {
  float estimatedDistance = 0.0;
  float targetDistance = 0.0;
  long maxTime = 11000; // 9 seconds
  float estimatedSpeed = 0.01;
  long startTimeDistance = millis();
  
  set_direction(FORWARDS);

  while (RUN && (millis() - startTimeDistance < maxTime)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  moving = false;
  // start finding can routine
  if (RUN) {
    vTaskResume(locateCanTask);
  }
  //suspend self
  vTaskSuspend(NULL);
}


// bluetooth coms task runs every 100 ms
void bluetoothTask(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim();
      if (command == "start") {
        SerialBT.println("Start Command Received");
        RUN = true;
        // makes sure the task doesnt start again while it is aleady moving
        if (!moving) {
          vTaskResume(ultrasoundTaskHandle);
          vTaskResume(moveToAreaTaskHandle);
          moving = true;
        }
      } else if (command == "stop") {
        SerialBT.println("Stop Command Received");
        RUN = false;
        stop_motors();
        moving = false;
      } else if (command == "ping") {
        SerialBT.println("Pong");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// loacte Can task
void locateCanTask(void *pvParameters) {
  const int step = 1;           // find resolution of servo
  const int delayBetweenSteps = 150; // ms
  int distanceF = 0;
  for (;;) {
    for (int angle = minUsUltra; angle <= maxUsUltra; angle += step) {
      servoUltrasound.writeMicroseconds(angle);
      vTaskDelay(pdMS_TO_TICKS(300));

      digitalWrite(trigPinF, LOW);
      delayMicroseconds(2);
  
      digitalWrite(trigPinF, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinF, LOW);
  
      vTaskDelay(pdMS_TO_TICKS(55));

      if (receivedF) {
        distanceF = (endTimeL - startTimeL) / 58;
        if (distanceF > 450) {
          distanceF = 450;
        }
        receivedF = false;
      }

      SerialBT.println(distanceF);

      if (!RUN) {
        vTaskSuspend(NULL);
      }
    }

    // suspend self
    vTaskSuspend(NULL);
  }
}

void setup() {
  //Serial.begin(115200); // Optional for Serial Monitor

  SerialBT.begin("ESP32_Rover");

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoSteering.setPeriodHertz(300);
  servoSteering.attach(steeringServoPin, minUsSteer, maxUsSteer);

  servoUltrasound.setPeriodHertz(300);   // change to match actual values
  servoUltrasound.attach(ultrasoundServoPin, minUsUltra, maxUsUltra);

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);

  attachInterrupt(digitalPinToInterrupt(echoPinL), echoL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinR), echoR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinF), echoF, CHANGE);
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
  vTaskSuspend(ultrasoundTaskHandle);

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

  // Create task to locate can 
  xTaskCreatePinnedToCore(
    locateCanTask,
    "locateCan",
    4000,
    NULL,
    1,
    NULL,
    1
  );
  vTaskSuspend(locateCanTask);

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
