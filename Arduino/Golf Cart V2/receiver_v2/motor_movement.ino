
int sgn(float num){
  if(num > 0) return 1;
  if(num < 0) return -1;
  return 0;
}

float kP = 2;
float kD = 0;
float kI = 0;

int accErrorCap = 1000;

int powerCap = 180;

float error, lastError = 0, accError = 0;
int headingPID(){
  error = orientation.yaw - targetHeading;

    // Heading wrapper
  if (error < -180)
      error = error + 360;
  else if (error > 180)
      error = error - 360;

  if(error > 1) accError += error;
  if(fabs(accError) > accErrorCap) accError = sgn(accError) * accErrorCap;
  
  float P = error * kP;
  float I = accError * kI;
  float D = lastError * kD;

  int turnPower = P + I + D;
  
  lastError = error;

  // Serial.println("error:" + (String) error);
  if(fabs(turnPower) > powerCap) turnPower = sgn(turnPower) * powerCap;

  return turnPower;
}

void balancePID(){
    
}