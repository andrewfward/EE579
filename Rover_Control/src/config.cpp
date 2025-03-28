#include "config.h"

// Constants
int minUs = 1100;
int maxUs = 1800;
float neutralPos = 1500.0;

// Servo objects
Servo servoSteering;
Servo servoUltrasound;

// Servo pin assignments
const int steeringServoPin = 18;
const int ultrasoundServoPin = 33;

// Right side ultrasonic sensor pins
const int trigPinR = 21;
const int echoPinR = 19;

// Left side ultrasonic sensor pins
const int trigPinL = 23;
const int echoPinL = 22;

// Front ultrasonic sensor pins
const int trigPinF = 16;
const int echoPinF = 15;

// Position and control variables
float pos = 0;
int landmarkCounter = -1;
bool landmarkFlag = false;

// Landmark distances
const float landmarkDistances[NUM_LANDMARKS] = {2.5, 3.0, 5.0};

// Initial offset for distance calculation
int initialOffset = 0;

// Steering angle for servo
int steeringAngle = 1500;

// Timing and state variables for ultrasound sensors
volatile long startTimeR = 0, endTimeR = 0;
volatile long startTimeL = 0, endTimeL = 0;
volatile bool receivedR = false, receivedL = false;

// Task handles
TaskHandle_t ultrasoundTaskHandle = NULL;
TaskHandle_t moveToAreaTaskHandle = NULL;

// Bluetooth Serial object (defined globally for use in other files)
BluetoothSerial SerialBT;