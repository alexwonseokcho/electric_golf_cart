#include "motion_control.hpp"
#include "esp_now.hpp"
#include "pins.hpp"
#include <Arduino.h>
#include "imu.hpp"

SemaphoreHandle_t requestedMovementMutex = NULL;
SemaphoreHandle_t appliedMovementMutex = NULL;
SemaphoreHandle_t speedReadingMutex = NULL; //not needed because double is atomically accessed?


movement_command requestedMovement = {0, 0};
movement_command appliedMovement = {0, 0};

double sgn(double num){
  if(num > 0) return 1.0;
  if(num < 0) return -1.0;
  return 0;
}


double incrementToAngle(double steerAngle, double angleTarget, double increment);
double getNewSteerInput(double lastAppliedAngle, double requestedAngle, double speedReading);
double deltaAngle(double angle1, double angle2);

drive_mode driveMode = RC_IMU;
void motionRequestTask(void *){
    delay(100); //make sure IMU task is already running before this task starts

    //takes in both remote control messages and the follow me, then consolidates it into the requestedMovement struct
    double requestedSpeed;    double requestedHeading; //[0, 360) degrees
    double steerAngle = 0;
    double _speedReading;

    double steerSensitivity = 0.001;
    int steerRange = 90;                       //+- this amount is the range


    if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE ){
      requestedHeading = IMU_data.heading;
      xSemaphoreGive(ImuMutex);
    }

    while(true){
        vTaskDelay(pdMS_TO_TICKS(0.5)); //the below has a chance (of 100%) of skipping the delay if it's after the continue 

        // if (millis() - remoteLastRecvTime > 2000) {
        //     requestedSpeed = 0;
        //     continue; //stop the cart if no signal is received, even in follow me mode
        // }

      if(driveMode == CALIBRATE_STEER){
        //constantly set the current position as steeroffset & allow the motor to move VERY slowly according to the input.
        //when the user switches out of this mode, the steeroffset will be correctly set --> that should be saved to eeprom
      }
      else if(driveMode == RC_IMU){

        requestedSpeed = incomingMessage.forwardSpeed * 1; //check if this needs to be semaphored
        int turn = incomingMessage.turn;

        if(speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE){
            _speedReading = speedReading;
            xSemaphoreGive(speedReadingMutex);
        }   

        //get the heading from the IMU
        double imuHeading;
        if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE ){
            imuHeading = IMU_data.heading;
            // Serial.println("TAKING FROM IMU: heading: " + String(IMU_data.heading) + " steerAngle: " + String(steerAngle));
            xSemaphoreGive(ImuMutex);
        }
        // imuHeading = 0; //remove this line when done testing


        if(deltaAngle(requestedHeading + sgn(turn) * steerSensitivity, imuHeading) < 90){
          //fixing the issue if the request outpaces the robot's change in heading, then it will wrap around 360, then command the opposite steer angle, then wrap around again
        
          requestedHeading += sgn(turn) * steerSensitivity; //maybe this should be based on the current heading from IMU. requested heading can be current heading +- sgn(turn) * wtv
        
          if(requestedHeading >= 360){
            requestedHeading -= 360;
          }
          if(requestedHeading < 0){
            requestedHeading += 360;
          }
        }

        //requestedHeading is the heading that the robot should be facing  
        steerAngle = requestedHeading - imuHeading;
        //take in the next angle function here to request exactly the max(or less) of what is possible to turn without instability
        if(steerAngle > 180){
          steerAngle -= 360;
        }
        else if(steerAngle < -180){
          steerAngle += 360;
        }

        //cap steerAngle at max steering range
        steerAngle = max(min(steerAngle, double(steerRange)), double(-steerRange));

        // Serial.println("steerAngle: " + String(steerAngle) + "imuHeading: " + String(imuHeading) + "requestedHeading: " + String(requestedHeading));
        
      }
      else if(driveMode == RC_RAW){
        requestedSpeed = incomingMessage.forwardSpeed * 10; //check if this needs to be semaphored
        int turn = incomingMessage.turn;


      }
      else if(driveMode == FOLLOW_ME){
        //if driveMode == FOLLOW_ME, the follow me task will begin writing to the requestedSpeed/Angle struct. Simply don't do anything? 
        //get the requested speed and angle from the follow task
        //requestedSpeed = followSpeed;
        //requestedAngle = followAngle;
      }
      else{
        requestedSpeed = 0;
        //do nothing

      }
      
      //set the requestedMovement struct
      if(requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE){
          requestedMovement.forwardSpeed = requestedSpeed;
          requestedMovement.steerAngle = steerAngle;
          // Serial.println("SET TO REQUESTED MOVEMENT: speed: " + String(requestedSpeed) + " angle: " + String(steerAngle));
          xSemaphoreGive(requestedMovementMutex);
      }
    }
    vTaskDelete( NULL );
}


bool isReverse(int vel) {
  if (vel < 0) return true;
  return false;
}

//void movement function should have higer priority and be able to shut off the motor as failsafe
void motionControlTask(void *){
  const double pidFrequency = 1000.0; //1kHz
  const double pidInterval = 1.0/pidFrequency; //s
  const double kP = 4, kI = 10, kD = 0.0;//kD = 0.08;
  double lastError = 0, error = 0, accumError = 0;
  double requestedSpeed = 0;
  double smoothedRequestedSpeed = 0;
  double requestedAngle = 0;
  double lastAppliedPower = 0;
  double lastAppliedAngle = 0;

  double powerIncrement = 0.1;
  double speedIncrement = 0.005;

  double steerOffset = -2; //this is the offset of the servo angle from the actual angle.

  /*
    shared state of requested speed and requested angle is used by this control code
  */
  while(true){
    if(requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE){
      requestedSpeed = requestedMovement.forwardSpeed;
      requestedAngle = requestedMovement.steerAngle;
      xSemaphoreGive(requestedMovementMutex);
      // Serial.println("requested speed: " + String(requestedSpeed) + " requested angle: " + String(requestedAngle));
    }
    else{
      Serial.println("Couldn't take the semaphore ");
      //this will just run it with repeated requested speed and angle values 
    }

    if (requestedSpeed > smoothedRequestedSpeed)
      smoothedRequestedSpeed = min(double(requestedSpeed), smoothedRequestedSpeed + speedIncrement);
    else if (requestedSpeed < smoothedRequestedSpeed)
      smoothedRequestedSpeed = max(double(requestedSpeed), double(smoothedRequestedSpeed - 2.0 * speedIncrement));


    if(speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE){
        error = smoothedRequestedSpeed - speedReading;
        xSemaphoreGive(speedReadingMutex);
    }
    double dPower = ((error - lastError) / pidInterval) * kD;
    double pidOutputPower = smoothedRequestedSpeed * 10.0 + error * kP + accumError * kI + ((error - lastError) / pidInterval) * kD;
    //check if error - lasterror has the right unit  

    pidOutputPower = sgn(pidOutputPower) * min(fabs(pidOutputPower), 100.0); //limit to +-255;

    lastError = error;  
    accumError += error * pidInterval; //[deg * S]

    if(smoothedRequestedSpeed == 0) accumError = 0; //reset the accumError if the requested speed is 0

    if(accumError > 10) accumError = 10;
    if(accumError < -10) accumError = -10;

    
    //acceleration limit
    
    if(appliedMovementMutex != NULL && xSemaphoreTake(appliedMovementMutex, portMAX_DELAY) == pdTRUE){
      lastAppliedPower = appliedMovement.forwardSpeed;
      lastAppliedAngle = appliedMovement.steerAngle;
      xSemaphoreGive(appliedMovementMutex);
    }

    double newAppliedPower;
    if (pidOutputPower > lastAppliedPower)
      newAppliedPower = min(double(pidOutputPower), lastAppliedPower + powerIncrement);
    else if (pidOutputPower < lastAppliedPower)
      newAppliedPower = max(double(pidOutputPower), double(lastAppliedPower - 0.8 * powerIncrement));

    double newSteerInput = getNewSteerInput(lastAppliedAngle, requestedAngle, speedReading);

    if(appliedMovementMutex != NULL && xSemaphoreTake(appliedMovementMutex, portMAX_DELAY) == pdTRUE){
      appliedMovement.forwardSpeed = newAppliedPower;
      appliedMovement.steerAngle = newSteerInput;
      xSemaphoreGive(appliedMovementMutex);
    }

    //output 
    // Serial.println("speed: " + String(speedReading) + " pidOutputPower: " + String(newAppliedPower) + "smoothed requested speed: " + String(smoothedRequestedSpeed) + " error: " + String(error) + " accumError: " + String(accumError) + " D POWER" + String(dPower) + " lastAppliedPower: " + String(lastAppliedPower) + " newAppliedPower: " + String(newAppliedPower) + " requestedAngle: " + String(requestedAngle) + " lastAppliedAngle: " + String(lastAppliedAngle) + " newAngle: " + String(newAngle));
    Serial.println("steerInput: " + String(newSteerInput));

    ledcWrite(1, abs(newAppliedPower) * 1023.0 / 255.0); //channel 1 is for motor
    digitalWrite(reversePin, isReverse(int(newAppliedPower)));
    int ticks = 2.53 * ((-(newSteerInput + steerOffset) + 90) * 270.0/180.0) + 171;
    Serial.println("ticks: " + String(ticks));
    ledcWrite(2,  ticks); //different for v3... !!! 2 is the channel for servo and 1 is the channel for motor


    // Serial.println("applied steer: " + String(appliedSteer));

    vTaskDelay(pdMS_TO_TICKS(pidInterval * 1000));
    yield();
  }
  vTaskDelete( NULL );
}


double incrementToAngle(double steerAngle, double angleTarget, double increment) {
  if (steerAngle < angleTarget) {
    steerAngle = min(steerAngle + increment, angleTarget);
  } else {
    steerAngle = max(steerAngle - increment, angleTarget);
  }
  return steerAngle;
}

double getNewSteerInput(double lastAppliedAngle, double requestedAngle, double speedReading){
  return requestedAngle;

  //max turn speed should be 0.25
  //CHANGE steerIncrementPerSpeedUnit TO WTV IS APPROPRIATE
  // //this should be moved to motion control task
  // double steerIncrement = max(0.25 - steerIncrementPerSpeedUnit * fabs(_speedReading), 0.07);  //steer speed varies linearly with speed with min steer speed = 0.1 when max speed
  // if (fabs(steerAngle) > 30) steerIncrement *= (1.5 - fabs(steerAngle) / steerRange);        //find a smoother turning profile that varies as a function of current speed and current angle
  // if (turn == -1) {
  //     steerAngle -= steerIncrement;
  //     if (steerAngle + steerOffset < 90 - steerRange) {  //steerAngle + steerOffset is [0, 180], so [steerRange, 180 - steerRange]
  //     steerAngle = 90 - steerRange - steerOffset;
  //     }
  //     if (fabs(_speedReading) > 30 && steerAngle + steerOffset < 90 - steerRange / 2.0) {
  //     int angleTarget = 90 - (steerRange / 2) - steerOffset;
  //     if (steerAngle < angleTarget) steerAngle = incrementToAngle(steerAngle, angleTarget, 2 * steerIncrement);
  //     else steerAngle = angleTarget;
  //     }
  // } else if (turn == 1) {
  //     steerAngle += steerIncrement;
  //     if (steerAngle + steerOffset > 90 + steerRange) {
  //     steerAngle = 90 + steerRange - steerOffset;
  //     }
  //     if (fabs(_speedReading) > 30 && steerAngle + steerOffset > 90 + steerRange / 2.0) {
  //     double angleTarget = (90 + steerRange / 2) - steerOffset;
  //     if (steerAngle > angleTarget)
  //         steerAngle = incrementToAngle(steerAngle, angleTarget, 2 * steerIncrement);
  //     else steerAngle = angleTarget;
  //     // if(steerAngle < )
  //     // steerAngle =  //bug where if angle is already past this, it'll jerk to this angle.
  //     }
  // } else {
  //     steerAngle = sgn(steerAngle) * max((fabs(steerAngle) - 1.5 * steerIncrement), 0.0);
  // }  
}

double deltaAngle(double angle1, double angle2){
  double diff = abs(angle1 - angle2);
  return min(diff, 360 - diff);
}