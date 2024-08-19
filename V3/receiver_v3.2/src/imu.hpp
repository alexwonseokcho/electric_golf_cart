
#ifndef _IMU_HPP_
#define _IMU_HPP_

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1


struct IMU_struct {
  double heading;
  double accX;
  double accY;
  double accZ;
};


struct euler_t {
  double yaw;
  double pitch;
  double roll;
};

extern SemaphoreHandle_t ImuMutex;
extern IMU_struct IMU_data; 

void IMUTask( void * );


void setReports(sh2_SensorId_t reportType, long report_interval, Adafruit_BNO08x bno08x);
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);
void getHeading(Adafruit_BNO08x bno08x, sh2_SensorValue_t sensorValue, euler_t* ypr);

#endif