#define INCLUDE_vTaskSuspend

#define DEBUG
#define PULSES_PER_REV 90.0 //change this
#define REV_PER_PULSE 1.0 / PULSES_PER_REV
#define WHEEL_DIAMETER 8 //inches
#define INCH_TO_KM 0.0000254
#define MICROSEC_TO_HOUR 0.00000000027778 

#define brakePin D7
#define reversePin D8
#define drivePin D9
#define steerServoPin D6
#define speedPin D10

const double TICK_TO_KM = (1.0 / REV_PER_PULSE) * WHEEL_DIAMETER * 3.141592 * INCH_TO_KM;
//delta is in microseconds & REV_PER_PULSE is in revs 

struct IMU_struct {
  double heading;
  double accX;
  double accY;
  double accZ;
};

struct movement_command {
  double forwardSpeed;
  double steerAngle; //this is centered around 0. The movement task will take this and convert to real angles from -135 to 135. 
};

volatile double speedReading; //this is in kph. in ISR, calculate this and put it up 

SemaphoreHandle_t ImuMutex = NULL;
SemaphoreHandle_t speedReadingMutex = NULL; //not needed because double is atomically accessed?
SemaphoreHandle_t requestedMovementMutex = NULL;
SemaphoreHandle_t appliedMovementMutex = NULL;
//use mutex for interrupt too

IMU_struct IMU_data; 
movement_command requestedMovement;
movement_command appliedMovement;

void IMUTask( void * );
void motionControl(void*);

#ifdef DEBUG
void monitorTask( void * );
#endif


unsigned long lastPulse = micros(); //this will only run the first time
void calcSpeed(){
  //check if static in an ISR is okay
  unsigned long currentTime = micros();
  int deltaT = currentTime - lastPulse;

  if(deltaT != 0){
    double newSpeed = TICK_TO_KM / double(deltaT * MICROSEC_TO_HOUR);

  // Serial.println(Hi );

  // if(speedReadingMutex != NULL && xSemaphoreTakeFromISR(speedReadingMutex, NULL)  == pdTRUE ) { //see if I can just try once then quit. Also look into pxHigherPriorityTaskWoken (right now it's just NULL)

  //NO NO THIS NEEDS TO BE AN ATOMIC UPDATE
  speedReading = speedReading * 0.8 + newSpeed * 0.2;
  }
    // xSemaphoreGiveFromISR(speedReadingMutex, NULL); //look into what to do with this HigherPriorityTaskWoken
  // }
  // else{
  //   // Serial.println("OH NO");
  // }

  lastPulse = currentTime;
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  speedReading = 0;
  requestedMovement = {0, 0};
  appliedMovement = {0, 0};

  // pinMode(speedPin, INPUT_PULLDOWN);
  pinMode(speedPin, INPUT_PULLUP);

  ImuMutex = xSemaphoreCreateMutex(); //create mutex
  speedReadingMutex = xSemaphoreCreateMutex();
  requestedMovementMutex = xSemaphoreCreateMutex();
  appliedMovementMutex = xSemaphoreCreateMutex();

  attachInterrupt(digitalPinToInterrupt(speedPin), calcSpeed, FALLING);


  xTaskCreate(IMUTask, //name of function
  "IMU Task", //name for my own sake
  2048, //stack size
  NULL, //if multiple tasks of the same function are started, we can keep track by passing the reference to a variable that uniquely keeps track of the task
  1, //priority of the task
  NULL //task handler, no need here
  //here, specify which core should run this 0 or 1
  );

  xTaskCreatePinnedToCore(motionControl, "PID and Acc Control", 2048, NULL, 1, NULL, 1); //look into how much stack I need for this


  //all tasks using speedReading should be on core 1??
  #ifdef DEBUG
  xTaskCreatePinnedToCore(monitorTask, "Monitor Task", 2048, NULL, 1, NULL, 1);
  #endif
}

void motionControl(void *){
  const double pidFrequency = 1000.0; //1kHz
  const double pidInterval = 1.0/pidFrequency; //s
  const double kP = 1, kI = 0, kD = 1;
  double lastError = 0, error = 0, accumError = 0;
  double requestedSpeed = 0;
  double requestedAngle = 0;
  double lastAppliedPower = 0;
  double lastAppliedAngle = 0;

  double powerIncrement = 0.25;
  /*
    shared state of requested speed and requested angle is used by this control code
  */
  while(true){
    if(requestedMovementMutex != NULL && xSemaphoreTake(requestedMovementMutex, portMAX_DELAY) == pdTRUE){
      requestedSpeed = requestedMovement.forwardSpeed;
      requestedAngle = requestedMovement.steerAngle;
      xSemaphoreGive(requestedMovementMutex);
    }
    else{
      Serial.println("Couldn't take the semaphore ");
      //this will just run it with repeated requested speed and angle values 
    }


    //do i need to take the mutex? double should be atomic 
    error = requestedSpeed - speedReading;
    
    double pidOutputPower = error * kP + accumError * kI + ((error - lastError) / pidInterval) * kD;
    //check if error - lasterror has the right unit  

    pidOutputPower = sgn(pidOutputPower) * min(fabs(pidOutputPower), 255.0); //limit to +-255;

    lastError = error; 
    accumError += error * pidInterval; //[deg * S]

    
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
      newAppliedPower = max(double(pidOutputPower), double(lastAppliedPower - 2.0 * powerIncrement));


    //make into a function: newAngle = f(lastAppliedAngle, speedReading, smth else)
    //todo: implement this function but for now,
    double newAngle = requestedAngle;

    if(appliedMovementMutex != NULL && xSemaphoreTake(appliedMovementMutex, portMAX_DELAY) == pdTRUE){
      appliedMovement.forwardSpeed = newAppliedPower;
      appliedMovement.steerAngle = newAngle;
      xSemaphoreGive(appliedMovementMutex);
    }

    //output 

    vTaskDelay(pdMS_TO_TICKS(pidInterval * 1000));
  }
}

//void movemeent function should have higer priority and be able to shut off the motor as failsafe

void monitorTask( void * ) {
  // configASSERT( ( ( uint32_t ) pvParameters ) == 1 ); //assert that the parameter is 1

  while(true){
    //lock mutex here
    // if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, ( TickType_t ) 20) == pdTRUE){ //wait 20 ticks for it to become available
    //   Serial.println("heading: " + String(IMU_data.heading) + " accX: " + String(IMU_data.accX) + " accY " + String(IMU_data.accY) + " accZ " + String(IMU_data.accZ));
    //   xSemaphoreGive(ImuMutex);
    // }
    // else{
    //   Serial.println("imu lock unsuccessful");
    // }
    if(speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, ( TickType_t ) 20) == pdTRUE){
      Serial.println("current speed: " + String(speedReading, 4));
      xSemaphoreGive(speedReadingMutex);
    }
    else{
      Serial.println("speed reading lock unsuccessful");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete( NULL );
}

void IMUTask( void * ) { //task for getting IMU data
  /* 
    initialize the IMU here
  */ 
  while(true){
    if(ImuMutex != NULL && xSemaphoreTake(ImuMutex, ( TickType_t ) 20) == pdTRUE ){ //wait 20 ticks for it to become available
      IMU_data.heading = random(200);
      IMU_data.accY = random(50);
      IMU_data.accX = random(20);
      IMU_data.accZ = random(5);
      xSemaphoreGive(ImuMutex);
    }
    else{
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(1));
}

double sgn(float num){
  if(num > 0) return 1.0;
  if(num < 0) return -1.0;
  return 0;
}