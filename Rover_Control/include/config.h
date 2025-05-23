#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// -------------------- definitions --------------------
#define SPEED_OF_SOUND 340
#define LEFT 1
#define RIGHT 0

// -------------------- Structures -------------------
// structure to store can values
struct scanValues {
  int angle;
  int distance;
};

extern scanValues scanData[29]; // store 29 values

// -------------------- Extern Variables --------------------
// Movement Flags
extern bool RUN;
extern bool moving;

// Runtime, default value is 9000 but can be set based on battery voltage.
extern int runtime;

// Servo Objects
extern Servo servoSteering;
extern Servo servoUltrasound;

// Bluetooth
extern BluetoothSerial SerialBT;

// Offsets calculated
extern bool offsetsCalculated;

// Servo Parameters
extern int minUsSteer;
extern int maxUsSteer;
extern float neutralPos;

// ultraound servo parameters 
extern int minUsUltra;
extern int maxUsUltra;

// Servo Pins
extern const int steeringServoPin;
extern const int ultrasoundServoPin;

// ESC pin
extern const int ESC_PWM_PIN;

// Ultrasound Sensor Pins
extern const int trigPinR;
extern const int echoPinR;
extern const int trigPinL;
extern const int echoPinL;
extern const int trigPinF;
extern const int echoPinF;

// Position and Control Variables
extern float pos;
extern float posR;
extern float posL;
extern int runtime;

// finding can variables 
extern int canAngle;
extern float currentCanDistance;

// Initial Offsets
extern int initialOffsetR;
extern int initialOffsetL;

// Steering
extern int steeringAngle;

// Ultrasound Timing
extern volatile long startTimeR;
extern volatile long endTimeR;
extern volatile long startTimeL;
extern volatile long endTimeL;
extern volatile long startTimeF;
extern volatile long endTimeF;
extern volatile bool receivedR;
extern volatile bool receivedL;
extern volatile bool receivedF;

// FreeRTOS Tasks
extern TaskHandle_t ultrasonicTaskHandle;
extern TaskHandle_t moveToAreaTaskHandle;
extern TaskHandle_t bluetoothTaskHandle;
extern TaskHandle_t locateCanTaskHandle;
extern TaskHandle_t driveToCanTaskHandle;
extern TaskHandle_t returnHomeTaskHandle;

#endif  // CONFIG_H
