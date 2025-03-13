#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#define SPEED_OF_SOUND 340

BluetoothSerial SerialBT;  // Bluetooth Serial object

// servo objects 
Servo servoSteering;
Servo servoUltrasound;

// min max values for the steering servo in us
int minUs = 1100;
int maxUs = 1800;
int neutralPos = 1500;

// servo pins 
const int steeringServoPin = 32;
const int ultrasoundServoPin = 33;

// right side ultrasound sensor
const int trigPinR = 16;
const int echoPinR = 17;

//left side ultrasound sensor
const int trigPinL = 18;
const int echoPinL = 19;

// front ultrasound sensor 
const int trigPinF = 23;
const int echoPinF = 15;

// the difference between dL and dR 
float pos = 0;

float distanceL = 0;
float distanceR = 0;

// calculated at the start such that dL - dR + offset = 0
float initialOffset = 0;

// control Parameters 
int Kp = 1;

int steeringAngle = 1500;

// timeing and state varaibles for ultrasound sensors
volatile long startTimeR = 0, endTimeR = 0;
volatile long startTimeL = 0, endTimeL = 0;
volatile bool receivedR = false, receivedL = false;

TaskHandle_t ultrasoundTaskHandle = NULL;


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

    pos = distanceL - distanceR + initialOffset;

    SerialBT.print("Lateral Position: ");
    SerialBT.println(pos);

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

void setup() {
    //Serial.begin(115200); // Start Serial Monitor

    ESP32PWM::allocateTimer(0);  // Allocate one timer for the steering 
    ESP32PWM::allocateTimer(1);  // Allocate one timer for the ultraounic sensor servo 

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
      
}

void loop() {

}