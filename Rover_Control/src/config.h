#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>  // Include Arduino framework
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

// Constants
#define SPEED_OF_SOUND 340
#define NUM_LANDMARKS 3

// Bluetooth Serial object
extern BluetoothSerial SerialBT;

// Servo objects
extern Servo servoSteering;
extern Servo servoUltrasound;

// Min/Max values for the steering servo in us
extern int minUs;
extern int maxUs;
extern float neutralPos;

// Servo pins
extern const int steeringServoPin;
extern const int ultrasoundServoPin;

// Right side ultrasound sensor
extern const int trigPinR;
extern const int echoPinR;

// ESC config
extern const int PWM_PIN;
extern const int PWM_CHANNEL;

// Left side ultrasound sensor
extern const int trigPinL;
extern const int echoPinL;

// Front ultrasound sensor
extern const int trigPinF;
extern const int echoPinF;

// Position and control variables
extern float pos;
extern int landmarkCounter;
extern bool landmarkFlag;

// Landmark distances
extern const float landmarkDistances[NUM_LANDMARKS];

// Initial offset for distance calculation
extern int initialOffset;

// Steering angle for the servo
extern int steeringAngle;

// Timing and state variables for ultrasound sensors
extern volatile long startTimeR;
extern volatile long endTimeR;
extern volatile long startTimeL;
extern volatile long endTimeL;
extern volatile bool receivedR;
extern volatile bool receivedL;

// Task handles
extern TaskHandle_t ultrasoundTaskHandle;
extern TaskHandle_t moveToAreaTaskHandle;

#endif  // CONFIG_H