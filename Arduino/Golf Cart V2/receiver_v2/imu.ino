#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

BNO08x IMU;

#define BNO08x_INT -1
#define BNO08x_RST -1

#define BNO08x_ADDR 0x4A

struct ypr {
  float yaw;
  float pitch;
  float roll;
};

ypr orientation;

//SDA D4, SCL D5
//TO DO: A LOT OF REDUNDANCY REQUIRED AND COMMUNICATE CLEARLY IF THIS FEATURE IS ON OR OFF
//TO DO: should be able to toggle on/off the IMU assist
//TO DO: Display should show if some parts of the cart are not working

void setupIMU(){
  
    Wire.begin();
    if(IMU.begin(BNO08x_ADDR, Wire, BNO08x_INT, BNO08x_RST) == false){
        Serial.println("IMU not found.");
        exceptionHandler();
    }

    setReports();
    delay(500);
    if(!(IMU.tareNow(true))){
        Serial.print ("Tare Failure"); 
        exceptionHandler();
    }
    orientation.pitch = 0; orientation.yaw = 0; orientation.roll = 0;
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (IMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
    exceptionHandler();
  }
  if (IMU.enableGravity() == true) {
    Serial.println(F("Gravity enabled"));
    Serial.println(F("Output in form x, y, z, accuracy"));
  } else {
    Serial.println("Could not enable gravity");
    exceptionHandler();
  }
}

void exceptionHandler(){ //make this usable with any other type of exception
    Serial.println("IMU Problem. Get Service from Alex. Restarting in 1 second.");
    //DISPLAY HERE ON THE SCREEN THAT THE IMU IS NOT WORKING
    delay(1000);
    ESP.restart();
};


void getIMUData(){
    if (IMU.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
    }

    if (IMU.getSensorEvent() == true) {
        if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            
            orientation.roll = (IMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
            orientation.pitch = (IMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
            orientation.yaw = (IMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
        }
        if (IMU.getSensorEventID() == SENSOR_REPORTID_GRAVITY) {
            float gravityX = IMU.getGravityX();
            float gravityY = IMU.getGravityY();
            float gravityZ = IMU.getGravityZ();
            float gravityAccuracy = IMU.getGravityAccuracy();

            // Serial.print(gravityX, 2);
            // Serial.print(F(","));
            // Serial.print(gravityY, 2);
            // Serial.print(F(","));
            // Serial.print(gravityZ, 2);
            // Serial.print(F(","));
            // Serial.print(gravityAccuracy, 2);
            // Serial.println();
        }
    }
}
