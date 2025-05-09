#include "esc_pwm.h"
#include "config.h"
#include "ultrasonic.h"
#include <cmath> // For abs() in C++
#include "main.h"

// Function declarations
void calculateInitialOffset(void);
void setOffsetBasedOnOneSide(bool);

int runtime = 9000;

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
  int distanceL = initialOffsetL;
  int distanceR = initialOffsetR;
  int prevDistanceR = 0;
  int prevDistanceL = 0;
  int maxChange = 8;

  // controller gains
  float Kp = 0.4;
  float Ki = 0.05;

  float error = 0.0;
  float eIntegral = 0.0;
  float timeStep = 65.0/1.0e3;

  // when false use left sensor when true use right 
  // this is to avoid them triggering each other
  bool toggleSensor = false;

  int lastL;
  int lastR; 

  int loopCount = 0;

  for (;;) {

    if (back == true) {
      Kp = 2.5;
      Ki = 0.05;
      maxChange = 20;
    }

    if (loopCount == 0) {
      lastL = initialOffsetL;
      lastR = initialOffsetR;
    }
    loopCount++;
    // logic to trigger the ultrasound sensors (bassed off datasheet)
    // uses the interrupts at the top
    if (toggleSensor == false) {
      distanceL = getUltrasoundValue(trigPinL);

      // adjusts offset if a large change is detected
      if (abs(distanceL - lastL) > maxChange) {
        int deltaL = distanceL - lastL;
        initialOffsetL += deltaL;
      }
      // calculates each offset because at one point I was seperatly handling the left and right sensors 
      // rather than combining then like it does now (functionally this is no different)
      posL = distanceL - initialOffsetL;
    } else {
      distanceR = getUltrasoundValue(trigPinR);
      // adjusts offset if a large change is detected
      if (abs(distanceR - lastR) > maxChange) {
        int deltaR = distanceR - lastR;
        initialOffsetR += deltaR;
      }
      posR = distanceR - initialOffsetR;
    }

    lastL = distanceL;
    lastR = distanceR;

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

// task to move to the general area of the can 
void moveToAreaTask(void *pvParameters) {
  // need to add routine to check the front sensor for colisions in case the timing is wrong
  float estimatedDistance = 0.0;
  float targetDistance = 0.0;

  // adjust to change how far it moves
  unsigned long maxTime = runtime; // normal operation is 9000 
  float estimatedSpeed = 0.01;
  unsigned long startTimeDistance = millis();
  
  set_direction(FORWARDS);

  // moves forward for maxTime unless stopped from interface
  while (RUN && ((millis() - startTimeDistance) < maxTime)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  SerialBT.println("CAN: Stopped moving");
  // start finding can routine if still in RUN
  if (RUN) {
    // added delay to allow rover to stop moving
    vTaskDelay(pdMS_TO_TICKS(1000));
    moving = false;
    vTaskResume(locateCanTaskHandle);
  }
  
  //suspend self
  vTaskSuspend(NULL);
}


// bluetooth coms task runs every 100 ms
void bluetoothTask(void *pvParameters) {
  for (;;) { 
    if (SerialBT.available()){
      String command = SerialBT.readStringUntil('\n');
      command.trim();
      if ((command == "start") && offsetsCalculated) {
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
      } else if (command == "calc_offsets") {
        calculateInitialOffset();      
        SerialBT.println("CAN: right offset: " + String(initialOffsetR));
        SerialBT.println("CAN: left offset: " + String(initialOffsetL));
        delay(1000);
        offsetsCalculated = true;
      } else if (command == "LHS") {
        setOffsetBasedOnOneSide(LEFT);
        SerialBT.println("CAN: right offset: " + String(initialOffsetR));
        SerialBT.println("CAN: left offset: " + String(initialOffsetL));
        offsetsCalculated = true;
      } else if (command == "RHS") {
        setOffsetBasedOnOneSide(RIGHT);
        SerialBT.println("CAN: right offset: " + String(initialOffsetR));
        SerialBT.println("CAN: left offset: " + String(initialOffsetL));
        offsetsCalculated = true;

      } else if (command == "BATTERY_HIGH") {
        runtime=6500; // milliseconds
        SerialBT.println("CAN: Battery high: set runtime to " + String(runtime/1000) + "s.");
      
      } else if (command == "BATTERY_MEDIUM") {
        runtime=7000;
        SerialBT.println("CAN: Battery medium: set runtime to " + String(runtime/1000) + "s.");
      
      } else if (command == "BATTERY_LOW") {
        runtime = 8000;
        SerialBT.println("CAN: Battery low: set runtime to " + String(runtime/1000) + "s.");

      } else if (command == "BATTERY_CRITICAL") {
        SerialBT.println("CAN: Battery needs recharged!");

        offsetsCalculated = true;
      }
    }
    // runs roughly every 100 ms
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// locate Can task
void locateCanTask(void *pvParameters) {
  bool canFound = false;
  const int step = 40;           
  int distanceF = 0;
  const int tolerance = 5;
  const int minSequence = 4;
  const int maxMismatches = 1;      // allowed mismatches in the sequence
  

  // structure to store can values
  struct scanValues {
    int angle;
    int distance;
  };

  scanValues scanData[29]; // store 61 values
  // outer for loop exists so that task can be resumed
  for (;;) {
    int count = 0;
    canFound = false;
    currentCanDistance = 400.0;
    bool minima = false;

    // sweeps the servo through 32 points
    // and takes an ultraound reading at each point
    for (int angle = minUsUltra; angle <= maxUsUltra; angle += step) {
      servoUltrasound.writeMicroseconds(angle);
      vTaskDelay(pdMS_TO_TICKS(350));

      distanceF = getUltrasoundValue(trigPinF);

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
        SerialBT.println("CAN: a sequence found");
        int beforeIdx = start - 1;
        int afterIdx = start + length;
        int sumAfter = 0;
        int sumBefore = 0;
        int countA = 0;
        int countB = 0;
        int averageAfter = 0;
        int averageBefore = 0;

        float minValValid = 450;
        // find average distance for comparison
        for (int j = start; j < afterIdx; j++) {
          if (scanData[j].distance < minValValid) {
            minValValid = scanData[j].distance;
          }
        }

        // calculate the average value after and before the sequence
        // made up of 3 values or however many there are (start of the array / end of the array)
        for (int k = afterIdx; k < afterIdx + 3; k++) {
          if (k >= count) {
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

        averageAfter = (countA > 0) ? (sumAfter / countA) : 0;
        averageBefore = (countB > 0) ? (sumBefore / countB) : 0;
        SerialBT.println("CAN: average after: " + String(averageAfter));
        SerialBT.println("CAN: average before: " + String(averageBefore));

        bool beforeHigher = (beforeIdx >= 0 && averageBefore > refDist + (tolerance));
        bool afterHigher = (afterIdx < count && averageAfter > refDist + (tolerance));

        // if it is a valid potentual can (minima)
        if ((beforeHigher && afterHigher) && minValValid < 100) {
          canFound = true;

          // this idicates that it is a minima not just one sided
          // this will give priority to sequences that are minima
          
          float minVal = 450;
          // find average distance for comparison
          for (int j = start; j < afterIdx; j++) {
            if (scanData[j].distance < minVal) {
              minVal = scanData[j].distance;
            }
          }
          // if less than the last found dip then replace (as more likely to be can)
          if (minima == false) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          } else if (minVal < currentCanDistance) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          }
          minima = true;
        // if only one sided and a true minima hasnt been found
        } else if (((beforeHigher || afterHigher) && minima == false) && minValValid < 100) {
          
          canFound = true;
          float minVal = 450;
          // find average distance for comparison
          for (int j = start; j < afterIdx; j++) {
            if (scanData[j].distance < minVal) {
              minVal = scanData[j].distance;
            }
          }
          // if less than the last found dip then replace (as more likely to be can)
          if (minVal < currentCanDistance) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          }
        }
        start += length;
      }
    }

    if (canFound == false) {
      SerialBT.println("CAN: noT can found");
      // add logic for failure to find can
      int maxSearchIncrement = 1000;
      int startSearchTime = millis();
      moving = true;

      // resume steering task
      vTaskResume(ultrasoundTaskHandle);
      vTaskDelay(pdMS_TO_TICKS(100));
      set_direction(FORWARDS);

      while(RUN && (millis()-startSearchTime)<maxSearchIncrement){
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      moving = false;
      stop_motors();
      // delay to allow ultrasound task to end
      vTaskDelay(pdMS_TO_TICKS(100));

    } else {
      SerialBT.println("CAN: Can Detected at angle: " + String(canAngle));
      if (currentCanDistance < 7.0) {
        SerialBT.println("CAN: Arrived at can");
        servoUltrasound.writeMicroseconds(1570);
        delay(2000);
        vTaskResume(returnHomeTaskHandle);
      } else {
        vTaskResume(driveToCanTaskHandle);
      }
      vTaskSuspend(NULL);
    }
    // suspend self
  }
}

// task to drive towards the can location found in the locateCanTask
void driveToCanTask(void *pvParameters) {
  unsigned long intervalTime;
  unsigned long startIntervalTime = -1;
  const int timeMultiplier = 150;
  const int timeOffset = 600;

  // this was guessed and can be adjusted as required
  const float servoRelation = 0.5;

  //infinite loop so that it can be resumed
  for (;;) {
    // sets the drive forward time as a function of the distance from the can
    intervalTime =  sqrt((int)currentCanDistance) * timeMultiplier;
    if (intervalTime < 600) {
      intervalTime = 600;
    }
    SerialBT.println("CAN: Interval time to drive towards can: " + String(intervalTime));
    // relates the angle of the ultrasound servo to the angle of the steering servo
    steeringAngle = neutralPos - ((1570 - canAngle)*servoRelation);

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


void returnHomeTask(void *pvParameters){
  unsigned long startReturnTime;
  unsigned long maxReturnTime = 600; // 9000;

  SerialBT.println("CAN: Returning home...");

  startReturnTime=millis();
  set_direction(BACKWARDS);
  moving = true;

  while(RUN && (millis()-startReturnTime < maxReturnTime)){
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_motors();
  moving = false;
  int loopCount = 0;

  for(;;) {
    int distanceL = getUltrasoundValue(trigPinL);
    int distanceR = getUltrasoundValue(trigPinR);
    if (distanceR < 50) {
      servoSteering.writeMicroseconds(1400);
    } else if (distanceL < 50) {
      servoSteering.writeMicroseconds(1520);
    } else {
      servoSteering.writeMicroseconds(1490);
    }
    maxReturnTime = 1000;
    startReturnTime=millis();
    moving = true;
    set_direction(BACKWARDS);

    while(RUN && (millis()-startReturnTime < maxReturnTime)){
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motors();
    moving = false;
    delay(500);

    if (loopCount > 7) {
      vTaskSuspend(NULL);
    }
    loopCount++;
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

  // calculateInitialOffset();
  // delay(1000);

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
  
  xTaskCreatePinnedToCore(
    returnHomeTask,
    "Return to start point after picking up coin",
    4000,
    NULL,
    1,
    &returnHomeTaskHandle,
    1
  );
  vTaskSuspend(returnHomeTaskHandle);

  delay(1000);
}

void loop() {
  // Nothing needed here
}
