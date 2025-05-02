#include "esc_pwm.h"
#include "config.h"
#include <cmath> // For abs() in C++

// Function declarations
void calculateInitialOffset();
float ultrasonicSensor(int trigPin);

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

// Tasks
void ultrasoundTask(void *pvParameters) {
  float prevPos = 0;
  float newPos = 0;
  int distanceL = 0;
  int distanceR = 0;
  int prevDistanceR = 0;
  int prevDistanceL = 0;

  // controller gains
  float Kp = 0.2;
  float Ki = 0.05;

  float error = 0.0;
  float eIntegral = 0.0;
  float timeStep = 65.0/1.0e3;

  // when false use left sensor when true use right 
  // this is to avoid them triggering each other
  bool toggleSensor = false;

  for (;;) {
    // logic to trigger the ultrasound sensors (bassed off datasheet)
    // uses the interrupts at the top
    if (toggleSensor == false) {
      distanceL = ultrasonicSensor(trigPinL);
      // calculates each offset because at one point I was seperatly handling the left and right sensors 
      // rather than combining then like it does now (functioanlly this is no different)
      posL = distanceL - initialOffsetL;
    } else {
      distanceR = ultrasonicSensor(trigPinR);
      posR = distanceR - initialOffsetR;
    }

    toggleSensor = !toggleSensor;

    // calculate pos based on the left and right psotion
    pos = posR - posL;

    // PI controller calculations 
    error = 0.0 - pos;
    eIntegral += error * timeStep;
    steeringAngle = (int)(neutralPos + (Kp * error) + (Ki * eIntegral));

    // bounds the steering value and applies it to the servo
    if (steeringAngle > maxUsSteer) steeringAngle = maxUsSteer;
    if (steeringAngle < minUsSteer) steeringAngle = minUsSteer;
    servoSteering.writeMicroseconds(steeringAngle);

    // logs the left and right distances, and the steering value and position
    String logEntry = String(distanceL) + "," + String(distanceR) + "," + String(steeringAngle) + "," + String(pos);
    SerialBT.println(logEntry);

    if (RUN == false || moving == false) {
      vTaskSuspend(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Function to read an ultrasonic sensor, selected using the corresponding trigPin
float ultrasonicSensor(int trigPin) {
  float distance = 0.0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(55));
  
  if ((trigPin == trigPinL) && receivedL) {
    distance = (endTimeL - startTimeL) / 58;
    receivedL = false;
  }
  else if ((trigPin == trigPinR) && receivedR) {
    distance = (endTimeR - startTimeR) / 58;
    receivedR = false;
  }
  else if ((trigPin == trigPinF) && receivedF) {
    distance = (endTimeF - startTimeF) / 58;
    receivedF = false;
  } else
  {
    SerialBT.println("CAN: ERROR - Invalid Ultrasonic Sensor State");
  }

  vTaskDelay(pdMS_TO_TICKS(5));
  if (distance > 450) {distance = 450;}
  return distance;
}


// Task to get the distances from the left and right ultrasonic sensors
void checkLRSensors(void *pvParameters) {
  int distanceL = 0;
  int distanceR = 0;

  // when false use left sensor when true use right 
  // this is to avoid them triggering each other
  bool toggleSensor = false;

  // logic to trigger the ultrasound sensors (based off datasheet)
  // uses the interrupts at the top
  if (toggleSensor == false) {
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    vTaskDelay(pdMS_TO_TICKS(55));
    if (receivedL) {
      distanceL = (endTimeL - startTimeL) / 58;
      if (distanceL > 450) {
        distanceL = 450;
      }
      receivedL = false;
    }

  } else {
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);

    // delay to wait for interupts to pick up the return signal 
    // if this is chnaged also need to chnage timeStep to match
    vTaskDelay(pdMS_TO_TICKS(60));
    if (receivedR) {
      distanceR = (endTimeR - startTimeR) / 58;
      if (distanceR > 450) {distanceR = 450;
      }
      receivedR = false;
    }
  }
    
  toggleSensor = !toggleSensor;

  // logs the left and right distances, and the steering value and position
  String logEntry = String(distanceL) + "," + String(distanceR);
  SerialBT.println("Left & Right Distance: "+ logEntry);

  if (RUN == false || moving == false) {
    vTaskSuspend(NULL);
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}

// task to move to the general area of the can 
void moveToAreaTask(void *pvParameters) {
  // need to add routine to check the front sensor for colisions in case the timing is wrong
  float estimatedDistance = 0.0;
  float targetDistance = 0.0;

  // adjust to change how far it moves
  unsigned long maxTime = 150; //for testing, normal operation is 9000 
  float estimatedSpeed = 0.01;
  unsigned long startTimeDistance = millis();
  
  set_direction(FORWARDS);

  // moves forward for maxTime unless stopped from interface
  while (RUN && ((millis() - startTimeDistance) < maxTime)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  SerialBT.println("CAN: Stopped moving");
  moving = false;
  // start finding can routine if still in RUN
  if (RUN) {
    vTaskResume(locateCanTaskHandle);
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
        // confirms connection and prints out the offsets
        SerialBT.println("Pong");
        SerialBT.println("CAN: right offset: " + String(initialOffsetR));
        SerialBT.println("CAN: left offset: " + String(initialOffsetL));
      }
    }
    // runs roughly every 100 ms
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// loacte Can task
void locateCanTask(void *pvParameters) {
  bool canFound = false;
  const int step = 40;           
  int distanceF = 0;
  const int tolerance = 5;
  const int minSequence = 5;
  const int maxMismatches = 1;      // allowed mismatches in the sequence

  // structure to store can values
  struct scanValues {
    int angle;
    int distance;
  };

  scanValues scanData[31]; // store 61 values
  // outer for loop exists so that task can be resumed
  for (;;) {
    int count = 0;
    canFound = false;
    currentCanDistance = 400.0;

    // sweeps the servo through 32 points
    // and takes an ultraound reading at each point
    for (int angle = minUsUltra; angle <= maxUsUltra; angle += step) {
      servoUltrasound.writeMicroseconds(angle);
      vTaskDelay(pdMS_TO_TICKS(300));

      digitalWrite(trigPinF, LOW);
      delayMicroseconds(2);
  
      digitalWrite(trigPinF, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinF, LOW);
  
      vTaskDelay(pdMS_TO_TICKS(80));

      if (receivedF) {
        distanceF = (endTimeF - startTimeF) / 58;
        if (abs(distanceF) > 450) {
          distanceF = 450;
        }
        receivedF = false;
      }

      scanData[count] = {angle, distanceF};

      // prints out the angle and distance to the GUI
      SerialBT.println(String(angle) + "," + String(distanceF));
      count++;

      if (!RUN) {
        vTaskSuspend(NULL);
      }
    }

    int midIdx = -1;
    // analyse data to find can
    for (int start = 0; start < count - minSequence; start++) {
      int refDist = scanData[start].distance;
      int length = 1;
      int mismatches = 0;
      for (int i = start + 1; i < count; i++) {
        if (abs(scanData[i].distance - refDist) <= tolerance) {
          length++;
        } else if (mismatches < maxMismatches){
          mismatches++;
        } else {
          break;
        }
      }

      // if the length of the sequence is greater than the min sequence length 
      // check if it is a valid sequence
      if (length >= minSequence) {
        int beforeIdx = start - 1;
        int afterIdx = start + length;
        int sumAfter = 0;
        int sumBefore = 0;
        int countA = 0;
        int countB = 0;
        int averageAfter = 0;
        int averageBefore = 0;

        // calculate the average value after and before the sequence
        // made up of 3 values or however many there are (start of the array / end of the array)
        for (int k = afterIdx; k < afterIdx + 3; k++) {
          if (k > count) {
            break;
          }
          sumAfter += scanData[k].distance;
          countA++;
        }

        for (int k = beforeIdx; k > beforeIdx - 3; k--) {
          if (k < 0) {
            break;
          }
          sumBefore += scanData[k].distance;
          countB++;
        }

        averageAfter = sumAfter / countA;
        averageBefore = sumBefore / countB;
        SerialBT.println("CAN: average after: " + String(averageAfter));
        SerialBT.println("CAN: average before: " + String(averageBefore));

        bool beforeHigher = (beforeIdx >= 0 && averageAfter > refDist + (tolerance));
        bool afterHigher = (afterIdx < count && averageBefore > refDist + (tolerance));

        // if it is a valid potentual can 
        if (beforeHigher || afterHigher) {
          canFound = true;
          float distanceSum = 0;
          float tempAverage = 0;
          // find average distance for comparison
          for (int j = start; j < (afterIdx - 1); j++) {
            distanceSum += scanData[j].distance;
          }
          tempAverage = distanceSum / (float)((afterIdx - 1) - start);
          // if less than the last found dip then replace (as more likely to be can)
          if (tempAverage < currentCanDistance) {
            currentCanDistance = tempAverage;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          }
          tempAverage = 0;
        }
        start += length;
      }
    }

    if (canFound == false) {
      SerialBT.println("CAN: no can found");
      // add logic for failure to find can
    } else {
      SerialBT.println("CAN: Can Detected at angle: " + String(canAngle));
      if (currentCanDistance < 10.0) {
        SerialBT.println("CAN: Arrived at can");
      } else {
        vTaskResume(driveToCanTaskHandle);
      }
    }
    // suspend self
    vTaskSuspend(NULL);
  }
}

void locateCanTaskV2(void *pvParameters) {
  bool canFound = false;
  const int step = 40;           
  int distanceF = 0;
  const int tolerance = 5;
  const int minSequence = 5;
  const int maxMismatches = 1;      // allowed mismatches in the sequence

  // structure to store can values
  struct scanValues {
    int angle;
    int distance;
  };

  scanValues scanData[31]; // store 61 values
  // outer for loop exists so that task can be resumed
  for (;;) {
    int count = 0;
    canFound = false;
    currentCanDistance = 400.0;

    // sweeps the servo through 32 points
    // and takes an ultraound reading at each point
    for (int angle = minUsUltra; angle <= maxUsUltra; angle += step) {
      servoUltrasound.writeMicroseconds(angle);
      vTaskDelay(pdMS_TO_TICKS(300));

      digitalWrite(trigPinF, LOW);
      delayMicroseconds(2);
  
      digitalWrite(trigPinF, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinF, LOW);
  
      vTaskDelay(pdMS_TO_TICKS(80));

      if (receivedF) {
        distanceF = (endTimeF - startTimeF) / 58;
        if (abs(distanceF) > 450) {
          distanceF = 450;
        }
        receivedF = false;
      }

      scanData[count] = {angle, distanceF};

      // prints out the angle and distance to the GUI
      SerialBT.println(String(angle) + "," + String(distanceF));
      count++;

      if (!RUN) {
        vTaskSuspend(NULL);
      }
    }

    int midIdx = -1;
    // analyse data to find can
    for (int start = 0; start < count - minSequence; start++) {
      int refDist = scanData[start].distance;
      int length = 1;
      int mismatches = 0;
      for (int i = start + 1; i < count; i++) {
        if (abs(scanData[i].distance - refDist) <= tolerance) {
          length++;
        } else if (mismatches < maxMismatches){
          mismatches++;
        } else {
          break;
        }
      }
    }
  }
}

// task to drive towards the can location found in the locateCanTask
void driveToCanTask(void *pvParameters) {
  unsigned long intervalTime;
  unsigned long startIntervalTime = -1;
  const int timeMultiplier = 12;
  const int timeOffset = 600;

  // this was guessed and can be adjusted as required
  const float servoRelation = 0.5;

  //infinite loop so that it can be resumed
  for (;;) {
    // sets the drive forward time as a function of the distance from the can
    intervalTime =  (((int)currentCanDistance) * timeMultiplier) + timeOffset;
    SerialBT.println("CAN: Interval time to drive towards can: " + String(intervalTime));
    // relates the angle of the ultrasound servo to the angle of the steering servo
    // 1500 is taken as the neutral angle for the ultrasound servo (this was never actuall tested to be exactly correct)
    steeringAngle = neutralPos - ((1500 - canAngle)*servoRelation);

    // sets bounds on the maximum steering angle
    if (steeringAngle > maxUsSteer) steeringAngle = maxUsSteer;
    if (steeringAngle < minUsSteer) steeringAngle = minUsSteer;

    // sets the steering servo angle
    servoSteering.writeMicroseconds(steeringAngle);
    startIntervalTime = millis();
    set_direction(FORWARDS);

    // moves froward for the intervalTime unless stopped from bluetooth interface
    while (RUN && ((millis() - startIntervalTime) < intervalTime)) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motors();
    vTaskResume(locateCanTaskHandle);
    // suspends self
    vTaskSuspend(NULL);
  }
}

void setup() {

  // begins the bluetooth interface
  SerialBT.begin("ESP32_Rover");

  // allocates timers for PWM for the ESC
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

  motorStartupSequence();
  delay(5000);

  calculateInitialOffset();
  delay(1000);

  // create tasks //
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
    &locateCanTaskHandle,
    1
  );
  vTaskSuspend(locateCanTaskHandle);

  // Create task to drive to can
  xTaskCreatePinnedToCore(
    driveToCanTask,
    "drive to can",
    4000,
    NULL,
    1,
    &driveToCanTaskHandle,
    1
  );
  vTaskSuspend(driveToCanTaskHandle);

  delay(1000);
}

void loop() {
  // Nothing needed here
}

void calculateInitialOffset() {
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
