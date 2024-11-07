#ifndef MOTION_CONTROL_HPP_
#define MOTION_CONTROL_HPP_

#include <Arduino.h>

extern double speedReading; //this is in kph. in ISR, calculate this and put it up 

struct movement_command {
  double leftSpeed;
  double rightSpeed;
  double accLimit;
};

enum drive_mode {
  RC_IMU,
  RC_RAW,
  FOLLOW_ME,
  STOP, //make this a thing too
};

extern SemaphoreHandle_t requestedMovementMutex;
extern SemaphoreHandle_t speedReadingMutex; //not needed because double is atomically accessed?

//use mutex for interrupt too

extern movement_command requestedMovement;

extern drive_mode driveMode;

void motionControlTask(void*);
void motionRequestTask(void *);

#endif