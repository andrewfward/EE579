#include <Arduino.h>
#include <NewPing.h>

#define MAX_DISTANCE_SIDE 400  // Maximum distance to measure on side sensors(in cm)
#define MAX_DISTANCE_FRONT 200  // Max distance to measure on front sensors (less so it doesnt pick up the wall behind)
#define US_NUMBER 3

// right side ultrasound sensor
const int TrigPinR = 16;
const int EchoPinR = 17;

//left side ultrasound sensor
const int TrigPinL = 18;
const int EchoPinL = 19;

// front ultrasound sensor 
const int TrigPinF = 23;
const int EchoPinF = 15;

NewPing USensors[US_NUMBER] = {   // Sensor object array.
  NewPing(TrigPinR, EchoPinR, MAX_DISTANCE_SIDE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TrigPinL, EchoPinL, MAX_DISTANCE_SIDE),
  NewPing(TrigPinF, EchoPinF, MAX_DISTANCE_FRONT)
};

TaskHandle_t ultrasoundTaskHandle = NULL;

void ultrasoundTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  // 60 ms (the recomended time in the dataSheet) 
  const TickType_t xFrequency = 60/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // delay until the set execution time (does not block the CPU)
    vTaskDelayUntil( &xLastWakeTime, xFrequency);

    // reads in the distance from both ultrasound sensors 
    // this might need to be seperated if each of these functions block unitl they get the data (they do :( )
    unsigned int distanceL = USensors[1].ping_cm();
    unsigned int distanceR = USensors[0].ping_cm();
  }
}

void setup() {
    Serial.begin(115200); // Start Serial Monitor

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