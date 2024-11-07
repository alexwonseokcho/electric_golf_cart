
#ifndef _IMU_HPP_
#define _IMU_HPP_

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1


struct IMU_struct {
  double heading;
  double pitch;
};


struct euler_t {
  double yaw;
  double pitch;
  double roll;
};

extern SemaphoreHandle_t ImuMutex;
extern IMU_struct IMU_data; 

void IMUTask( void * );

#endif