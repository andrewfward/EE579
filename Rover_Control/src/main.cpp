#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "pwm.h"
#include <cmath> // For abs() in C++

#define SPEED_OF_SOUND 340
#define NUM_LANDMARKS 3

BluetoothSerial SerialBT;  // Bluetooth Serial object

// servo objects 
Servo servoSteering;
Servo servoUltrasound;

// min max values for the steering servo in us
int minUs = 1100;
int maxUs = 1800;
int neutralPos = 1500;

// servo pins 
const int steeringServoPin = 18;
const int ultrasoundServoPin = 33;

// right side ultrasound sensor
const int trigPinR = 23;
const int echoPinR = 22;

//left side ultrasound sensor
const int trigPinL = 21;
const int echoPinL = 19;

// front ultrasound sensor 
const int trigPinF = 16;
const int echoPinF = 15;

// the difference between dR and dL 
float pos = 0;

float distanceL = 0;
float distanceR = 0;
float prevDistanceR = 0;
int landmarkCounter = -1;
bool landmarkFlag = false;

// placeholder values at the moment
const float landmarkDistances[NUM_LANDMARKS] = {2.5, 3.0, 5.0};

// calculated at the start such that dR - dL + offset = 0
float initialOffset = 0;

// control Parameters 
int Kp = 1;

int steeringAngle = 1500;

// timeing and state varaibles for ultrasound sensors
volatile long startTimeR = 0, endTimeR = 0;
volatile long startTimeL = 0, endTimeL = 0;
volatile bool receivedR = false, receivedL = false;

TaskHandle_t ultrasoundTaskHandle = NULL;
TaskHandle_t moveToAreaTaskHandle = NULL;

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

    pos = distanceR - distanceL + initialOffset;

    prevDistanceR = distanceR;

    SerialBT.print("Lateral Position: ");
    SerialBT.println(pos);

    // add function to detect landmarks (ie dips in the corridor)
    if (abs(distanceR - prevDistanceR) > 20) {
      landmarkCounter += 1;
      landmarkFlag = true;
    }

    // control code to convert that pos to an angle (1100 - 1800)
    // where 1500 is 0
    // starts with an simple proportional controller

    steeringAngle = (int)(neutralPos + Kp * pos);

    SerialBT.print("Steering Angle (us): ");
    SerialBT.println(steeringAngle);

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
    
    calculateInitialOffset();

    // code for ESC setup routine
    //motorStartupSequence();    
}

void loop() {

}

void calculateInitialOffset() {
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

  initialOffset = distanceL - distanceR;

  SerialBT.print("Initial Offset (cm): ");
  SerialBT.println(initialOffset);
}