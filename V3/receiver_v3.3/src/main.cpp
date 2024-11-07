#include <Arduino.h>
#include "imu.hpp"
#include "motion_control.hpp"
#include "odometer.hpp"
#include "esp_now.hpp"
#include "follow.hpp"
#include "config.hpp"

#ifndef INCLUDE_vTaskSuspend
  #define INCLUDE_vTaskSuspend
#endif

// #define DEBUG

//V3.3 has odrive, V3.2 has raw motor driver

#ifdef DEBUG
void monitorTask( void * );
#endif

void setup() {
  delay(100);
  Serial.begin(115200);
  

  if (!ledcSetup(2, 344, 10)) { 
    ESP.restart();
  }
  ledcAttachPin(steerServoPin, 2); //pin number, channel

  /*
  if (!ledcAttachChannel(steerServoPin, 344, 10, 2)) {
    ESP.restart();
  }
  */

  ImuMutex = xSemaphoreCreateMutex(); //create mutex
  requestedMovementMutex = xSemaphoreCreateMutex();
  speedReadingMutex = xSemaphoreCreateMutex();

  espNowSetup();


  Serial.println("Starting...");

  xTaskCreatePinnedToCore(IMUTask, //name of function
  "IMU Task", //name for my own sake
  2048, //stack size
  NULL, //if multiple tasks of the same function are started, we can keep track by passing the reference to a variable that uniquely keeps track of the task
  1, //priority of the task
  NULL, //task handler, no need here
  1//here, specify which core should run this 0 or 1
  );
  delay(1000); //wait for IMU to start up before starting the other tasks instead of this dumb thing I did above
  xTaskCreatePinnedToCore(motionControlTask, "PID and Acc Control", 20480, NULL, 1, NULL, 1); //look into how much stack I need for this
  delay(500);
  xTaskCreatePinnedToCore(motionRequestTask, "Motion Request Task", 2048, NULL, 1, NULL, 1);
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

    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete( NULL );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1));
}
