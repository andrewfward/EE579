#include "config.h"

// -------------------- Logging --------------------
LogEntry logData[MAX_LOG_ENTRIES];
int logIndex = 0;
bool logDataReady = false;

// -------------------- Movement Flags --------------------
bool RUN = false;
bool moving = false;

// -------------------- Servo Objects --------------------
Servo servoSteering;
Servo servoUltrasound;

// -------------------- Bluetooth --------------------
BluetoothSerial SerialBT;

// -------------------- Servo Parameters --------------------
int minUs = 1200;
int maxUs = 1700;
float neutralPos = 1500.0;

// -------------------- Servo Pins --------------------
const int steeringServoPin = 18;
const int ultrasoundServoPin = 33;

// -------------------- Ultrasound Sensor Pins --------------------
const int trigPinR = 21;
const int echoPinR = 19;

const int trigPinL = 23;
const int echoPinL = 22;

const int trigPinF = 16;
const int echoPinF = 15;

// -------------------- Position and Control Variables --------------------
float pos = 0;
float posR = 0;
float posL = 0;
int landmarkCounter = -1;
bool landmarkFlag = false;

// -------------------- Landmark Distances --------------------
const float landmarkDistances[NUM_LANDMARKS] = {2.5, 3.0, 5.0};

// -------------------- Initial Offsets --------------------
int initialOffsetR = 0;
int initialOffsetL = 0;

// -------------------- Steering --------------------
int steeringAngle = 1500;

// -------------------- Ultrasound Timing --------------------
volatile long startTimeR = 0;
volatile long endTimeR = 0;
volatile long startTimeL = 0;
volatile long endTimeL = 0;
volatile bool receivedR = false;
volatile bool receivedL = false;

// -------------------- FreeRTOS Task Handles --------------------
TaskHandle_t ultrasoundTaskHandle = NULL;
TaskHandle_t moveToAreaTaskHandle = NULL;
TaskHandle_t bluetoothTaskHandle = NULL;
