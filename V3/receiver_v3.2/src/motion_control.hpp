#ifndef MOTION_CONTROL_HPP_
#define MOTION_CONTROL_HPP_

#include <Arduino.h>

extern volatile double speedReading; //this is in kph. in ISR, calculate this and put it up 

struct movement_command {
  double forwardSpeed;
  double steerAngle; //this is centered around 0. The movement task will take this and convert to real angles from -135 to 135. 
};

enum drive_mode {
  RC_IMU,
  RC_RAW,
  FOLLOW_ME,
  CALIBRATE_STEER,
  STOP, //make this a thing too
};

extern SemaphoreHandle_t requestedMovementMutex;
extern SemaphoreHandle_t appliedMovementMutex;
extern SemaphoreHandle_t speedReadingMutex; //not needed because double is atomically accessed?

//use mutex for interrupt too

extern movement_command requestedMovement;
extern movement_command appliedMovement;

extern drive_mode driveMode;

void motionControlTask(void*);
void motionRequestTask(void *);

#endif