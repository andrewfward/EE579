#include <Arduino.h>
#include "BluetoothSerial.h"

#define SPEED_OF_SOUND 340

BluetoothSerial SerialBT;  // Bluetooth Serial object

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

TaskHandle_t ultrasoundTaskHandle = NULL;

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
      float distanceL = (endTimeL - startTimeL)/58;
      SerialBT.print("Left Ultrasonic Sensor: ");
      SerialBT.println(distanceL);
      receivedL = false;
    }

    if (receivedR) {
      float distanceR = (endTimeR - startTimeR)/58;
      SerialBT.print("Left Ultrasonic Sensor: ");
      SerialBT.println(distanceR);
      receivedR = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // wait max time for signals to be recived
  }
}

void setup() {
    Serial.begin(115200); // Start Serial Monitor

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