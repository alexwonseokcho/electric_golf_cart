#include "imu.hpp"
#include <Arduino.h>
#include "config.hpp"

SemaphoreHandle_t ImuMutex = NULL;
IMU_struct IMU_data;

Adafruit_BNO08x bno08x(BNO08X_RESET); //maybe move this to imu_functions.cpp and only require sensorValue to be here
sh2_SensorValue_t sensorValue;
euler_t ypr; // yaw pitch roll

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000; //set this lower (check datasheet)

void linearAccelToVel(sh2_Accelerometer* linearAcceleration);

void IMUTask( void * ) { //task for getting IMU data
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    delay(500);
    ESP.restart(); //still doesn't work after resetting
  }
  Serial.println("BNO08x Found!");

  setReports(SH2_ARVR_STABILIZED_RV, reportIntervalUs, bno08x);
  setReports(SH2_LINEAR_ACCELERATION, reportIntervalUs, bno08x);
  

  Serial.println("Reading events");

  while(true){
    if (bno08x.getSensorEvent(&sensorValue))
    {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId)
      {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
          break;
        case SH2_LINEAR_ACCELERATION:
          linearAccelToVel(&sensorValue.un.linearAcceleration);
          break;
        
      }
      static long last = 0;
      long now = micros();
      // Serial.print(now - last);
      // Serial.print("\t");
      last = now;
      // Serial.print("accuracy of heading: " + (String) sensorValue.status);
      // Serial.print("\t"); // This is accuracy in the range of 0 to 3


      //also get the acceleration values (or the velocity if that's given)
      
      double heading = -ypr.yaw;      
      if(heading < 0.0) heading += 360.0;
      // Serial.println("yaw pitch roll: " + String(ypr.yaw) + " " + String(ypr.pitch) + " " + String(ypr.roll));
// xSemaphoreTake(requestedMovementMutex, portMAX_DELAY)
      if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, portMAX_DELAY) == pdTRUE ){ //wait 20 ticks for it to become available
        IMU_data.heading = heading;
        IMU_data.accY = 0; //make this velocity probably
        IMU_data.accX = 0;
        IMU_data.accZ = 0;
        // Serial.println("heading: " + String(IMU_data.heading));
        xSemaphoreGive(ImuMutex);
      }
    
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
  vTaskDelete( NULL );
}

void setReports(sh2_SensorId_t reportType, long report_interval, Adafruit_BNO08x bno08x)
{
   Serial.println("Setting desired reports");
   if (!bno08x.enableReport(reportType, report_interval)) Serial.println("Could not enable stabilized remote vector");
}

// Function to multiply two quaternions
void multiplyQuaternions(
    float w1, float x1, float y1, float z1, // First quaternion (w1, x1, y1, z1)
    float w2, float x2, float y2, float z2, // Second quaternion (w2, x2, y2, z2)
    float &wResult, float &xResult, float &yResult, float &zResult // Result quaternion
) {
    // Perform quaternion multiplication
    wResult = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    xResult = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    yResult = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    zResult = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  // Variables to hold the result
  float wResult, xResult, yResult, zResult;

  // Multiply the quaternions            w i j k
  //for simple 135 y axis down, just get the quaternion for -135 to bring it back to robot frame. ask chatGPT for help if transofrm changes lol 
  //TODO: yaw flips sign crazy when the imu goes fully upside down... prob gimbal lock but yeah that's not good for the algorithm 
  // multiplyQuaternions(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, 0.7071, 0, -0.7071, 0, wResult, xResult, yResult, zResult);
  multiplyQuaternions(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, 0.3826834, 0.9238795, 0, 0, wResult, xResult, yResult, zResult);
  
  quaternionToEuler(wResult, xResult, yResult, zResult, ypr, degrees);
  // Serial.println("yaw pitch roll: " + String(ypr->yaw) + " " + String(ypr->pitch) + " " + String(ypr->roll));

}


void linearAccelToVel(sh2_Accelerometer* linearAcceleration){
  // Serial.println(String(linearAcceleration->x) + " " + String(linearAcceleration->y) + " " + String(linearAcceleration->z));
}
