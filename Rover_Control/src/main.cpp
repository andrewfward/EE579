#include <Arduino.h>

#define SPEED_OF_SOUND 340

// right side ultrasound sensor
const int trigPinR = 16;
const int echoPinR = 17;

//left side ultrasound sensor
const int trigPinL = 18;
const int echoPinL = 19;

// front ultrasound sensor 
const int trigPinF = 23;
const int echoPinF = 15;

// timeing and state varaibles for ultrasound sensors
volatile long startTimeR = 0, endTimeR = 0;
volatile long startTimeL = 0, endTimeL = 0;
volatile bool receivedR = false, receivedL = false;

// Interrupt service routines for echo signals
void IRAM_ATTR echoR_start() { startTimeR = micros(); }
void IRAM_ATTR echoR_end() { endTimeR = micros(); receivedR = true; }

void IRAM_ATTR echoL_start() { startTimeL = micros(); }
void IRAM_ATTR echoL_end() { endTimeL = micros(); receivedL = true; }

TaskHandle_t ultrasoundTaskHandle = NULL;

void ultrasoundTask(void *pvParameters) {
  for (;;) {
    digitalWrite(trigPinR, LOW);
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
    // send trigger pulse for 10 us 
    digitalWrite(trigPinR, HIGH);  
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);  
    digitalWrite(trigPinL, LOW);

    vTaskDelay(pdMS_TO_TICKS(20)); // wait max time for signals to be recived

    if (receivedR) {
      float distanceR = ((endTimeR - startTimeR)/1000000) * SPEED_OF_SOUND/2;
      Serial.print("Left Ultrasonic Sensor: ");
      Serial.println(distanceR);
      receivedR = false;
    }

    if (receivedL) {
      float distanceL = ((endTimeL - startTimeL)/1000000) * SPEED_OF_SOUND/2;
      Serial.print("Left Ultrasonic Sensor: ");
      Serial.println(distanceL);
      receivedR = false;
    }
  }
}

void setup() {
    Serial.begin(115200); // Start Serial Monitor

    pinMode(trigPinR, OUTPUT);
    pinMode(echoPinR, INPUT);
    pinMode(trigPinL, OUTPUT);
    pinMode(echoPinL, INPUT);
    pinMode(trigPinF, OUTPUT);
    pinMode(echoPinF, INPUT);
    
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