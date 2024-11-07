#include "motion_control.hpp"
#include "esp_now.hpp"
#include "config.hpp"
#include <Arduino.h>
#include "imu.hpp"
#include "utils.hpp"
#include <ODriveUART.h>
#include <HardwareSerial.h>

SemaphoreHandle_t requestedMovementMutex = NULL;
SemaphoreHandle_t speedReadingMutex = NULL; // not needed because double is atomically accessed?

double speedReading = 0; // this is in kph

movement_command requestedMovement = {0.0, 0.0, MAX_ACC / 2.0};

HardwareSerial odrive_serial_left(0);
HardwareSerial odrive_serial_right(1);
ODriveUART odrive_left(odrive_serial_left);
ODriveUART odrive_right(odrive_serial_right);

double deltaAngle(double angle1, double angle2);
void sendOdrive(double leftSpeed, double rightSpeed, double accLimit);
double wrap(double angle);
double calculateMaxAcc(double pitch);

drive_mode driveMode = RC_IMU;
void motionRequestTask(void *)
{
  delay(100); // make sure IMU task is already running before this task starts

  // takes in both remote control messages and the follow me, then consolidates it into the requestedMovement struct
  double requestedSpeed = 0;
  double targetHeading;
  double imuHeading; //[0, 360) degrees
  double imuPitch = 0.0;
  double steerSpeed = 0;
  double maxSteerSpeed = 3; //3 KPH
  double _speedReading;
  double accLimit = MAX_ACC / 2.0; // turns/s^2

  double steerSensitivity = 0.001; // factor of how much the remote control wants the cart to turn

  if (ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE)
  {
    targetHeading = IMU_data.heading;
    xSemaphoreGive(ImuMutex);
  }

  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(0.5)); // the below has a chance (of 100%) of skipping the delay if it's after the continue

    if (millis() - remoteLastRecvTime > 3000)
    {
      incomingMessage.forwardSpeed = 0;
    }
    
    if (driveMode == RC_IMU)
    {

      requestedSpeed = incomingMessage.forwardSpeed * 1; // check if this needs to be semaphored
      int turn_dir = incomingMessage.turn;


      if (speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE)
      {
        _speedReading = speedReading;
        xSemaphoreGive(speedReadingMutex);
      }

      // get the heading from the IMU
      if (ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE)
      {
        imuHeading = IMU_data.heading;
        imuPitch = IMU_data.pitch;
        xSemaphoreGive(ImuMutex);
      }

      if(turn_dir == 0) 
        steerSpeed = 0;
      else{
        // steerSpeed += steerSensitivity * turn_dir;
        // steerSpeed = sgn(steerSpeed) * min(maxSteerSpeed, abs(steerSpeed));
        steerSpeed = sgn(turn_dir) * 1.0;
      }

      accLimit = calculateMaxAcc(imuPitch);

      // Serial.println("imu pitch: " + String(imuPitch) + " accLimit: " + String(accLimit));
      // Serial.println("req speed: " + String(requestedSpeed) + "cur speed: " + String(_speedReading) + " steerSpeed: " + String(steerSpeed) + " targetHeading: " + String(targetHeading) + " imuHeading: " + String(imuHeading));
    }
    else if (driveMode == RC_RAW)
    {
      requestedSpeed = incomingMessage.forwardSpeed * 10; // check if this needs to be semaphored
      int turn = incomingMessage.turn;
    }
    else if (driveMode == FOLLOW_ME)
    {
      // if driveMode == FOLLOW_ME, the follow me task will begin writing to the requestedSpeed/Angle struct. Simply don't do anything?
      // get the requested speed and angle from the follow task
      // requestedSpeed = followSpeed;
      // requestedAngle = followAngle;
      //still use accLimit = calculateMaxAcc(_speedReading, IMU_data.pitch);
    }
    else
    {
      requestedSpeed = 0;
      // do nothing
    }



    // set the requestedMovement struct
    if (requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE)
    {
      requestedMovement.leftSpeed = requestedSpeed + steerSpeed;
      requestedMovement.rightSpeed = requestedSpeed - steerSpeed;
      requestedMovement.accLimit = accLimit;
      // Serial.println("SET TO REQUESTED MOVEMENT: speed: " + String(requestedSpeed) + " angle: " + String(steerSpeed));
      xSemaphoreGive(requestedMovementMutex);
    }
  }
  vTaskDelete(NULL);
}

// void movement function should have higer priority and be able to shut off the motor as failsafe
void motionControlTask(void *)
{

  double leftSpeed = 0;
  double rightSpeed = 0;
  double accLimit = MAX_ACC / 2.0; // turns/s^2

  odrive_serial_left.begin(115200, SERIAL_8N1, ODR_RX_LEFT, ODR_TX_LEFT);
  odrive_serial_right.begin(115200, SERIAL_8N1, ODR_RX_RIGHT, ODR_TX_RIGHT);

  delay(10);

  while (odrive_left.getState() == AXIS_STATE_UNDEFINED)
  {
    delay(100);
    Serial.println("waiting for odrive left to connect");
  }

  while (odrive_right.getState() == AXIS_STATE_UNDEFINED)
  {
    delay(100);
    Serial.println("waiting for odrive right to connect");
  }


  while (odrive_left.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrive_left.clearErrors();
    odrive_left.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.println("left odrive setting changed to closed loop control");
    delay(10);
  }
  while (odrive_right.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrive_right.clearErrors();
    odrive_right.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.println("right odrive setting changed to closed loop control");
    delay(10);
  }

  /*
    shared state of requested speed and requested angle is used by this control code
  */
  while (true)
  {
    if (requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE)
    {
      leftSpeed = requestedMovement.leftSpeed;
      rightSpeed = requestedMovement.rightSpeed;
      accLimit = requestedMovement.accLimit;
      xSemaphoreGive(requestedMovementMutex);
    }

  

    sendOdrive(leftSpeed, rightSpeed, accLimit);
    yield();
  }

  vTaskDelete(NULL);
}

double deltaAngle(double angle1, double angle2)
{
  double diff = abs(angle1 - angle2);
  return min(diff, 360 - diff);
}

char output[50];
void sendOdrive(double leftSpeed, double rightSpeed, double accLimit)
{
  // requestedSpeed is in kph, and odrive outputs with turns/s
  sprintf(output, "w axis0.controller.config.vel_ramp_rate %.3f\n", accLimit);
  odrive_serial_left.write(output);
  odrive_serial_right.write(output);

  odrive_left.setVelocity(-leftSpeed / TURN_PER_SEC_TO_KPH);
  odrive_right.setVelocity(rightSpeed / TURN_PER_SEC_TO_KPH);

  ODriveFeedback feedback_left = odrive_left.getFeedback();
  ODriveFeedback feedback_right = odrive_right.getFeedback();

  // Serial.print("pos:");
  // Serial.print(feedback_left.pos);
  // Serial.print(", ");

  // Serial.print("DC voltage: ");
  // Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  // Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_ramp_rate"));

  if (speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE)
  {
    speedReading = -((feedback_left.vel + (-feedback_right.vel)) / 2.0) * TURN_PER_SEC_TO_KPH;
    Serial.println("left: " + String(leftSpeed) + " right: " + String(rightSpeed) + " accLimit: " + String(accLimit) + " speed: " + String(speedReading));
    xSemaphoreGive(speedReadingMutex);
  }
}

double wrap(double angle)
{
  if (angle >= 360)
  {
    return angle - 360;
  }
  if (angle < 0)
  {
    return angle + 360;
  }
  return angle;
}

double calculateMaxAcc(double pitch)
{
  if(pitch < 0.0) return MAX_ACC;
  else return max(MIN_ACC, min(MAX_ACC, MAX_ACC - ((MAX_ACC - MIN_ACC)/ MAX_PITCH_FOR_MIN_ACC) * pitch));
}