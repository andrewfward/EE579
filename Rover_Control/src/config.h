#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>



// -------------------- Constants --------------------
#define SPEED_OF_SOUND 340
#define NUM_LANDMARKS 3
#define MAX_LOG_ENTRIES 500

// -------------------- Structures --------------------
struct LogEntry {
    int distanceL;
    int distanceR;
    int steering;
};

// -------------------- Extern Variables --------------------

// Logging
extern LogEntry logData[MAX_LOG_ENTRIES];
extern int logIndex;
extern bool logDataReady;

// Movement Flags
extern bool RUN;
extern bool moving;

// Servo Objects
extern Servo servoSteering;
extern Servo servoUltrasound;

// Bluetooth
extern BluetoothSerial SerialBT;

// Servo Parameters
extern int minUs;
extern int maxUs;
extern float neutralPos;

// Servo Pins
extern const int steeringServoPin;
extern const int ultrasoundServoPin;

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
extern int landmarkCounter;
extern bool landmarkFlag;

// Landmark Distances
extern const float landmarkDistances[NUM_LANDMARKS];

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
extern volatile bool receivedR;
extern volatile bool receivedL;

// FreeRTOS Tasks
extern TaskHandle_t ultrasoundTaskHandle;
extern TaskHandle_t moveToAreaTaskHandle;
extern TaskHandle_t bluetoothTaskHandle;

#endif  // CONFIG_H
