#include "config.h"


// -------------------- Movement Flags --------------------
bool RUN = false;
bool moving = false;

// -------------------- Servo Objects --------------------
Servo servoSteering;
Servo servoUltrasound;

// -------------------- Bluetooth --------------------
BluetoothSerial SerialBT;

// -------------- Check if offsets have been calculated --------------
bool offsetsCalculated = false;

// -------------------- can analysis structure --------------------
scanValues scanData[29]; // store 29 values

// -------------------- Servo Parameters --------------------

int minUsSteer = 1140;
int maxUsSteer = 1800;
float neutralPos = 1450.0;

int minUsUltra = 970;
int maxUsUltra = 2100;

// -------------------- Servo Pins--------------------
const int steeringServoPin = 32;
const int ultrasoundServoPin = 33;

// -------------------- ESC pin--------------------
const int ESC_PWM_PIN = 14;

// -------------------- Ultrasound Sensor Pins --------------------
const int trigPinL = 16;
const int echoPinL = 17;

const int trigPinR = 26;
const int echoPinR = 27;

const int trigPinF = 23;
const int echoPinF = 22;

// -------------------- Position and Control Variables --------------------
float pos = 0;
float posR = 0;
float posL = 0;
int runtime = 9000;

// -------------------- finding can variables --------------------
int canAngle = -1;
float currentCanDistance = 400.0;

// -------------------- Initial Offsets --------------------
int initialOffsetR = 0;
int initialOffsetL = 0;

// -------------------- Steering --------------------
int steeringAngle = 1450;

// -------------------- Ultrasound Timing --------------------
volatile long startTimeR = 0;
volatile long endTimeR = 0;
volatile long startTimeL = 0;
volatile long endTimeL = 0;
volatile long startTimeF = 0;
volatile long endTimeF = 0;
volatile bool receivedR = false;
volatile bool receivedL = false;
volatile bool receivedF = false;

// -------------------- FreeRTOS Task Handles --------------------
TaskHandle_t ultrasoundTaskHandle = NULL;
TaskHandle_t moveToAreaTaskHandle = NULL;
TaskHandle_t bluetoothTaskHandle = NULL;
TaskHandle_t locateCanTaskHandle = NULL;
TaskHandle_t driveToCanTaskHandle = NULL;
TaskHandle_t returnHomeTaskHandle = NULL;