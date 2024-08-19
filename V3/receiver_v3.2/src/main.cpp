#include <Arduino.h>
#include "imu.hpp"
#include "motion_control.hpp"
#include "odometer.hpp"
#include "esp_now.hpp"
#include "follow.hpp"
#include "pins.hpp"

#ifndef INCLUDE_vTaskSuspend
  #define INCLUDE_vTaskSuspend
#endif

// #define DEBUG

#ifdef DEBUG
void monitorTask( void * );
#endif

double sgn(float num);

void hallAISR(){
  //determine direction by checking if A and B are equal 
  int direction = (digitalRead(hallBPin) == digitalRead(hallAPin)) ? -1 : 1;
  tickCount += direction; 
}

void hallBISR(){
  int direction = (digitalRead(hallBPin) == digitalRead(hallCPin)) ? -1 : 1;
  tickCount += direction; 
}

void hallCISR(){
  int direction = (digitalRead(hallCPin) == digitalRead(hallAPin)) ? -1 : 1;
  tickCount += direction; 
}

volatile double speedReading = 0;

void setup() {
  delay(100);
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(hallAPin, INPUT_PULLDOWN);
  pinMode(hallBPin, INPUT_PULLDOWN);
  pinMode(hallCPin, INPUT_PULLDOWN);

  pinMode(drivePin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  // pinMode(brakePin, OUTPUT);
  
  if (!ledcSetup(1, 30000, 10)) {//channel, freq, resolution
    ESP.restart();
  }
  ledcAttachPin(drivePin, 1);

  if (!ledcSetup(2, 344, 10)) { 
    ESP.restart();
  }
  ledcAttachPin(steerServoPin, 2); //pin number, channel

  /* FOR VERSION 3.0.0+ OF ARDUINO ESP32 CORE
  if (!ledcAttachChannel(drivePin, 4000, 10, 1)) { //pin, freq, resolution, channel
    ESP.restart();
  }
  if (!ledcAttachChannel(steerServoPin, 344, 10, 2)) {
    ESP.restart();
  }
  */

  ImuMutex = xSemaphoreCreateMutex(); //create mutex
  speedReadingMutex = xSemaphoreCreateMutex();
  requestedMovementMutex = xSemaphoreCreateMutex();
  appliedMovementMutex = xSemaphoreCreateMutex();

  espNowSetup();


  //enable only one at a time to test them
  attachInterrupt(digitalPinToInterrupt(hallAPin), hallAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallBPin), hallBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallCPin), hallCISR, CHANGE);

  Serial.println("Starting...");

  xTaskCreatePinnedToCore(IMUTask, //name of function
  "IMU Task", //name for my own sake
  2048, //stack size
  NULL, //if multiple tasks of the same function are started, we can keep track by passing the reference to a variable that uniquely keeps track of the task
  1, //priority of the task
  NULL, //task handler, no need here
  1//here, specify which core should run this 0 or 1
  );
  delay(2000); //wait for IMU to start up before starting the other tasks instead of this dumb thing I did above
  xTaskCreatePinnedToCore(motionRequestTask, "Motion Request Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(motionControlTask, "PID and Acc Control", 2048, NULL, 1, NULL, 1); //look into how much stack I need for this
  xTaskCreatePinnedToCore(odometerTask, "Odometer Task", 2048, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore(followTask, "Follow Task", 2048, NULL, 1, NULL, 0);
  #ifdef DEBUG
  xTaskCreatePinnedToCore(monitorTask, "Monitor Task", 2048, NULL, 1, NULL, 1);
  #endif
}

void monitorTask( void * ) {
  // configASSERT( ( ( uint32_t ) pvParameters ) == 1 ); //assert that the parameter is 1

  while(true){
    // lock mutex here
    if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, ( TickType_t ) 20) == pdTRUE){ //wait 20 ticks for it to become available
      Serial.println("heading: " + String(IMU_data.heading) + " accX: " + String(IMU_data.accX) + " accY " + String(IMU_data.accY) + " accZ " + String(IMU_data.accZ));
      xSemaphoreGive(ImuMutex);
    }
    else{
      Serial.println("imu lock unsuccessful");
    }
    if(speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, ( TickType_t ) 20) == pdTRUE){
      Serial.println("current speed: " + String(speedReading, 4) + "tick count: " + String(tickCount  ));
      xSemaphoreGive(speedReadingMutex);
    }
    else{
      Serial.println("speed reading lock unsuccessful");
    }
    if(requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, ( TickType_t ) 20) == pdTRUE){
      Serial.println("req forw: " + String(requestedMovement.forwardSpeed, 4) + " req steer: " + String(requestedMovement.steerAngle, 4));
      xSemaphoreGive(requestedMovementMutex);
    }
    else{
      Serial.println("requested movement lock unsuccessful");
    }
    if(appliedMovementMutex != NULL && xSemaphoreTake(appliedMovementMutex, ( TickType_t ) 20) == pdTRUE){
      Serial.println("app forw: " + String(appliedMovement.forwardSpeed, 4) + " app steer: " + String(appliedMovement.steerAngle, 4));
      xSemaphoreGive(appliedMovementMutex);
    }
    else{
      Serial.println("applied movement lock unsuccessful");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete( NULL );
}


void loop() {
  // put your main code here, to run repeatedly:
  // ledcWrite(drivePin, 100);
  vTaskDelay(pdMS_TO_TICKS(1));
}
