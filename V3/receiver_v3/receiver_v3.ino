#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// #define brakePin 10
// #define reversePin 11
// #define drivePin 12
// #define steerServoPin 6
#define brakePin D7
#define reversePin D8
#define drivePin D9
#define steerServoPin D6

typedef struct incoming_message{
  int16_t forwardSpeed;
  int16_t steeringAngle;
} incoming_message;

incoming_message incomingMessage;

int remoteLastRecvTime;

void espNowSetup(){
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}


void setup() {
  espNowSetup();
  remoteLastRecvTime = millis();
  Serial.begin(9600);

  pinMode(drivePin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(brakePin, OUTPUT);

  incomingMessage.forwardSpeed = 0;
  incomingMessage.steeringAngle = 0;

  if(!ledcAttachChannel(drivePin, 4000, 10, 1)){
    ESP.restart();
  }
  if(!ledcAttachChannel(steerServoPin, 344, 10, 2)){
    ESP.restart();
  }
}


float smallSteerSpeedLimit = 20; //when steering less than 15 degrees, you don't need to slow down. if you do, however, 
                                  //slow down to 20 (whatever units this is lol) before making any further turns

float appliedSpeed = 0;
float speedIncrement = 0.25;
float steerIncrement = 0.25;
int steerDeadzone = 45;
int zeroSteerOffset = -3; //offset 
float servoAngle = 135 + zeroSteerOffset;
void loop() {

  float requestedServoAngle = map(-incomingMessage.steeringAngle, -135, 135, steerDeadzone + zeroSteerOffset, 270 - steerDeadzone + zeroSteerOffset);
  int requestedSpeed = incomingMessage.forwardSpeed;

  // if(abs(-incomingMessage.steeringAngle + zeroSteerOffset) > 15){
  //   requestedSpeed = smallSteerSpeedLimit; //slow down when turning
  // }

  if(millis() - remoteLastRecvTime > 2000){
    requestedSpeed = 0;
  }

  if(requestedSpeed > appliedSpeed)
    appliedSpeed = min(float(requestedSpeed), appliedSpeed + speedIncrement);
  else if(requestedSpeed < appliedSpeed)
    appliedSpeed = max(double(requestedSpeed), appliedSpeed - 2.0 * speedIncrement);
  
  if(abs(appliedSpeed) > 255) //CAP SPEED 
    appliedSpeed = sgn(appliedSpeed) * 255;

  // if(abs(-incomingMessage.steeringAngle + zeroSteerOffset) < 15){
    if(requestedServoAngle > servoAngle)
      servoAngle = min(requestedServoAngle, servoAngle + steerIncrement);
    else if(requestedServoAngle < servoAngle)
      servoAngle = max(requestedServoAngle, servoAngle - steerIncrement);
  // }

  // if(appliedSpeed > smallSteerSpeedLimit)
  //   servoAngle = sgn(servoAngle) * max(fabs(servoAngle), float(15.0));

  //CHECK IF HEART BEAT CHECK AUTO SHUTOFF WORKS BY CUTTING POWER TO REMOTE WITH MOTOR SPINNING
  //TO DO: CHECK IF STEER ACCELERATION WORKS PROPERLY (KEEP ANGLE AT 135 AND THEN CUT POWER TO REMOTE, THEN RECONNECT SO IT JUMPS TO 0)
  //CHECK IF SLOWED DOWN TURN WORKS --> IT SHOULD TURN TO MAX OF 15 DEGS AND IT WILL ONLY PROCEED MAKING A BIGGER TURN IF SPEED IS BELOW 20 (WILL START SLOWING DOWN HOPEFULLY)
  //CHECK STEER DEADZONE FUNCTIONALITY 

  // if(fabs(appliedSpeed) < 0.01){
  //   Serial.println("BRAKE PIN HIGH");
  //   digitalWrite(brakePin, HIGH);
  // }
  // else{
  //   digitalWrite(brakePin, LOW);
  // }

  //do a PID on the speed output of the motor for holding purposes (maybe not with the brake gimmick)
  digitalWrite(reversePin, !isReverse(appliedSpeed));
  ledcWrite(drivePin, abs(appliedSpeed) * 1023 / 255);
  ledcWrite(steerServoPin, servoAngle * 900 / 270);

  Serial.print("Applied speed: ");
  Serial.println(appliedSpeed);
  Serial.print("Steering angle: ");
  Serial.println(incomingMessage.steeringAngle);
  Serial.print("Requested Steering Angle: ");
  Serial.println(requestedServoAngle);
  Serial.print("Applied Steering Angle: ");
  Serial.println(servoAngle);
  delay(5);
}

bool isReverse(int vel) {
  if(vel < 0) return true;
  return false;
} 

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data) {
  remoteLastRecvTime = millis();
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  digitalWrite(LED_BUILTIN, LOW);
}

int sgn(int velocity){
  if(velocity > 0) return 1;
  else if(velocity < 0) return -1;
  return 0;
}