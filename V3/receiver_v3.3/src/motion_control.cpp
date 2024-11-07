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

movement_command requestedMovement = {0, 0};

HardwareSerial odrive_serial(1);
ODriveUART odrive(odrive_serial);

double getNewSteerInput(double lastAppliedAngle, double requestedAngle, double speedReading);
double deltaAngle(double angle1, double angle2);
void sendOdrive(double requestedSpeed, double acc_limit);
double maxDeltaSteerAngle(int turn_dir, double speedReading, double steerAngle, int steerRange);
double wrap(double angle);
double maxSteerRange(double speedReading);

drive_mode driveMode = RC_IMU;
void motionRequestTask(void *)
{
  delay(100); // make sure IMU task is already running before this task starts

  // takes in both remote control messages and the follow me, then consolidates it into the requestedMovement struct
  double requestedSpeed;
  double targetHeading;
  double imuHeading; //[0, 360) degrees
  double steerAngle = 0;
  double steerAngleSmoothed = 0;
  double _speedReading;
  double lastAngle = 0;

  double steerSensitivity = 0.8; // factor of how much the remote control wants the cart to turn
  double steerRange = 0;           //+- this amount is the range

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

    if (driveMode == CALIBRATE_STEER)
    {
      // constantly set the current position as steeroffset & allow the motor to move VERY slowly according to the input.
      // when the user switches out of this mode, the steeroffset will be correctly set --> that should be saved to eeprom
    }
    else if (driveMode == RC_IMU)
    {

      requestedSpeed = incomingMessage.forwardSpeed * 1.25; // check if this needs to be semaphored
      int turn_dir = incomingMessage.turn;


      if (speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE)
      {
        _speedReading = speedReading;
        xSemaphoreGive(speedReadingMutex);
      }

      double maxDeltaTheta = maxDeltaSteerAngle(turn_dir, _speedReading, steerAngle, steerRange);
      steerRange = maxSteerRange(_speedReading);

      // get the heading from the IMU
      if (ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE)
      {
        imuHeading = IMU_data.heading;
        // Serial.println("TAKING FROM IMU: heading: " + String(IMU_data.heading) + " steerAngle: " + String(steerAngle));
        xSemaphoreGive(ImuMutex);
      }

      if (false /*resetButtonPressed*/)
      {
        targetHeading = imuHeading;
      }
      else if (deltaAngle(targetHeading, imuHeading) > steerRange)
      {
        if (deltaAngle(wrap(targetHeading + turn_dir), imuHeading) < deltaAngle(targetHeading, imuHeading))
        {
          targetHeading = wrap(steerAngle + imuHeading);
          // if the target heading is already at or past the maximum ster servo range and the user commands it such that it reduces delta
          // then immediately change the targetHeading to max steerRange to ensure the system is responsive (rather than slowly winding the angle back to servo-able position)
        }
      }
      else
      {
        // change heading normally here
        targetHeading += sgn(turn_dir) * maxDeltaTheta * steerSensitivity;
        targetHeading = wrap(targetHeading);
      }

      steerAngle = targetHeading - imuHeading;
      if(steerAngle > 180) steerAngle -= 360;
      if(steerAngle < -180) steerAngle += 360;

      // cap steerAngle at max steering range
      steerAngle = max(min(steerAngle, double(steerRange)), double(-steerRange));

      // limit the change in steer angle to the maxDeltaSteerAngle
      steerAngleSmoothed = max(min(steerAngle, lastAngle + maxDeltaTheta), lastAngle - maxDeltaTheta);


      //update lastAngle
      lastAngle = steerAngleSmoothed;

      Serial.println("steerAngle: " + String(steerAngle) + " steerAngleSmoothed: " + String(steerAngleSmoothed) + " targetHeading: " + String(targetHeading) + " imuHeading: " + String(imuHeading));
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
    }
    else
    {
      requestedSpeed = 0;
      // do nothing
    }

    // set the requestedMovement struct
    if (requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE)
    {
      requestedMovement.forwardSpeed = requestedSpeed;
      requestedMovement.steerAngle = steerAngleSmoothed;
      // Serial.println("SET TO REQUESTED MOVEMENT: speed: " + String(requestedSpeed) + " angle: " + String(steerAngle));
      xSemaphoreGive(requestedMovementMutex);
    }
  }
  vTaskDelete(NULL);
}

// void movement function should have higer priority and be able to shut off the motor as failsafe
void motionControlTask(void *)
{

  double speedOutput = 0;
  double steerAngleOutput = 0;

  double steerOffset = -2; // this is the offset of the servo angle from the actual angle.

  odrive_serial.begin(115200, SERIAL_8N1, ODR_RX, ODR_TX);

  delay(10);
  while (odrive.getState() == AXIS_STATE_UNDEFINED)
  {
    delay(100);
  }

  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }

  double acc_limit = ACC_LIMIT; // turns/s^2 TODO: put into config

  /*
    shared state of requested speed and requested angle is used by this control code
  */
  while (true)
  {
    if (requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE)
    {
      speedOutput = requestedMovement.forwardSpeed;
      steerAngleOutput = requestedMovement.steerAngle;
      xSemaphoreGive(requestedMovementMutex);
      // Serial.println("requested speed: " + String(requestedSpeed) + " requested angle: " + String(requestedAngle));
    }

  

    int ticks = 2.53 * ((-(steerAngleOutput + steerOffset) + 90) * 270.0 / 180.0) + 171;
    ledcWrite(2, ticks); // different for v3... !!! 2 is the channel for servo and 1 is the channel for motor
    sendOdrive(speedOutput, acc_limit);
    Serial.println("im alive");
    yield();
  }

  vTaskDelete(NULL);
}

unsigned long lastTime = 0;
double maxDeltaSteerAngle(int turn_dir, double speedReading, double steerAngle, int steerRange)
{
  double delta;
  unsigned long curTime = micros();
  if(lastTime == 0) delta = 0;
  else delta = (curTime - lastTime) / 1000000.0 ;

  lastTime = curTime;

  if (steerAngle <= -steerRange && turn_dir == -1)
    return 0;
  else if (steerAngle >= steerRange && turn_dir == 1)
    return 0;
  else
    return delta * max(100.0 - 7.0 * fabs(speedReading), 15.0);
  //deg/s --> TODO: put into config
}

double deltaAngle(double angle1, double angle2)
{
  double diff = abs(angle1 - angle2);
  return min(diff, 360 - diff);
}

void sendOdrive(double requestedSpeed, double acc_limit)
{
  // requestedSpeed is in kph, and odrive outputs with turns/s

  char output[60];
  sprintf(output, "w axis0.controller.config.vel_ramp_rate %.3f\n", acc_limit); // check if this is actually 3 decimal points
  // Serial.println(output);
  odrive_serial.write(output);

  odrive.setVelocity(-requestedSpeed / TURN_PER_SEC_TO_KPH);

  ODriveFeedback feedback = odrive.getFeedback();
  // Serial.print("pos:");
  // Serial.print(feedback.pos);
  // Serial.print(", ");

  // Serial.println("requestedSpeed: " + String(requestedSpeed) + " speedReading: " + String(speedReading));
  // Serial.print("DC voltage: ");
  // Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  // Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_ramp_rate"));

  // utils to do turns/s to kph

  if (speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE)
  {
    speedReading = -feedback.vel * TURN_PER_SEC_TO_KPH;
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

double maxSteerRange(double speedReading)
{
  //60 at standstill, 48 at 4 kph 30 at 30 kph
  // return max(60.0 - 3 * fabs(speedReading), 10.0); //degrees 
  //now, make it 80 at standstill and 52 at 4 kph
  return max(75.0 - 4.5 * fabs(speedReading), 10.0);

}