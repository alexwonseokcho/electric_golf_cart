#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// #define brakePin 10
// #define reversePin 11
// #define drivePin 12
// #define steerServoPin 6
// #define speedPin 9
#define brakePin D7
#define reversePin D8
#define drivePin D9
#define steerServoPin D6
#define speedPin D10

typedef struct incoming_message {
  int16_t forwardSpeed;
  int8_t turn;
  int16_t remoteBatteryVoltage;
  bool set_zero_steer;
} incoming_message;

incoming_message incomingMessage;

int remoteLastRecvTime;

uint8_t espNowBroadcastAddress[] = { 0x84, 0xfc, 0xe6, 0x6b, 0x87, 0x08 };
esp_now_peer_info_t peerInfo;

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}


void espNowSetup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    ESP.restart();
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
}


void setup() {
  espNowSetup();
  remoteLastRecvTime = millis();
  Serial.begin(9600);

  pinMode(drivePin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(brakePin, OUTPUT);

  incomingMessage.forwardSpeed = 0;
  incomingMessage.turn = 0;

  if (!ledcAttachChannel(drivePin, 4000, 10, 1)) {
    ESP.restart();
  }
  if (!ledcAttachChannel(steerServoPin, 344, 10, 2)) {
    ESP.restart();
  }
}


float smallSteerSpeedLimit = 20;  //when steering less than 15 degrees, you don't need to slow down. if you do, however,
                                  //slow down to 20 (whatever units this is lol) before making any further turns

float appliedSpeed = 0;
float speedIncrement = 0.25;
float steerIncrementPerSpeedUnit = 0.005;  //--> at 20 speed, I want 0.2 turn increment so 0.01 * 20 --> 0.2
int steerRange = 45;                       //+- this amount is the range
float steerAngle = 0;
float steerOffset = 91;  //degrees
void loop() {
  int requestedSpeed = incomingMessage.forwardSpeed * 7;
  int turn = incomingMessage.turn;

  if (millis() - remoteLastRecvTime > 2000) {
    requestedSpeed = 0;
  }

  if (requestedSpeed > appliedSpeed)
    appliedSpeed = min(float(requestedSpeed), appliedSpeed + speedIncrement);
  else if (requestedSpeed < appliedSpeed)
    appliedSpeed = max(double(requestedSpeed), appliedSpeed - 2.0 * speedIncrement);

  //max is 255 but i'm limiting it to 150 to be safe. 
  if (abs(appliedSpeed) > 150)  //CAP SPEED
    appliedSpeed = sgn(appliedSpeed) * 150;

  //max turn speed should be 0.25
  float steerIncrement = max(0.25 - steerIncrementPerSpeedUnit * fabs(appliedSpeed), 0.07);  //steer speed varies linearly with speed with min steer speed = 0.1 when max speed
  if (fabs(steerAngle) > 30) steerIncrement *= (1.5 - fabs(steerAngle) / steerRange);        //find a smoother turning profile that varies as a function of current speed and current angle
  if (turn == -1) {
    steerAngle -= steerIncrement;
    if (steerAngle + steerOffset < 90 - steerRange) {  //steerAngle + steerOffset is [0, 180], so [steerRange, 180 - steerRange]
      steerAngle = 90 - steerRange - steerOffset;
    }
    if (fabs(appliedSpeed) > 30 && steerAngle + steerOffset < 90 - steerRange / 2.0) {
      int angleTarget = 90 - (steerRange / 2) - steerOffset;
      if (steerAngle < angleTarget) steerAngle = incrementToAngle(steerAngle, angleTarget, 2 * steerIncrement);
      else steerAngle = angleTarget;
    }
  } else if (turn == 1) {
    steerAngle += steerIncrement;
    if (steerAngle + steerOffset > 90 + steerRange) {
      steerAngle = 90 + steerRange - steerOffset;
    }
    if (fabs(appliedSpeed) > 30 && steerAngle + steerOffset > 90 + steerRange / 2.0) {
      float angleTarget = (90 + steerRange / 2) - steerOffset;
      if (steerAngle > angleTarget)
        steerAngle = incrementToAngle(steerAngle, angleTarget, 2 * steerIncrement);
      else steerAngle = angleTarget;
      // if(steerAngle < )
      // steerAngle =  //bug where if angle is already past this, it'll jerk to this angle.
    }
  } else {
    steerAngle = sgn(steerAngle) * max((fabs(steerAngle) - 1.5 * steerIncrement), 0.0);
  }
  //FIX THIS --> MAKE IT SO THAT WHEN THIS MODE IS ENABLED, THE REMOTE WILL STEER THE WHEEL VERY SLOWLY AND U CAN LOCK IT IN WITH THE SAME COMBINATION OF BUTTONS AGAIN
  if (incomingMessage.set_zero_steer) steerOffset = steerOffset + steerAngle;  //this is the new "zero point"

  //steer angle is -steerrange to +steerrange

  // if(abs(-incomingMessage.steeringAngle + zeroSteerOffset) < 15){
  // if(requestedServoAngle > servoAngle)
  //   servoAngle = min(requestedServoAngle, servoAngle + steerIncrement);
  // else if(requestedServoAngle < servoAngle)
  //   servoAngle = max(requestedServoAngle, servoAngle - steerIncrement);
  // }

  // if(appliedSpeed > smallSteerSpeedLimit)
  //   servoAngle = sgn(servoAngle) * max(fabs(servoAngle), float(15.0));

  //CHECK IF HEART BEAT CHECK AUTO SHUTOFF WORKS BY CUTTING POWER TO REMOTE WITH MOTOR SPINNING
  //TO DO: CHECK IF STEER ACCELERATION WORKS PROPERLY (KEEP ANGLE AT 135 AND THEN CUT POWER TO REMOTE, THEN RECONNECT SO IT JUMPS TO 0)
  //CHECK IF SLOWED DOWN TURN WORKS --> IT SHOULD TURN TO MAX OF 15 DEGS AND IT WILL ONLY PROCEED MAKING A BIGGER TURN IF SPEED IS BELOW 20 (WILL START SLOWING DOWN HOPEFULLY)
  //CHECK STEER DEADZONE FUNCTIONALITY

  //do a PID on the speed output of the motor for holding purposes (maybe not with the brake gimmick)
  digitalWrite(reversePin, !isReverse(appliedSpeed));
  //don't reverse it if speed is still high...
  ledcWrite(drivePin, abs(appliedSpeed) * 1023 / 255);

  float appliedSteer = (steerOffset + steerAngle) * 135 / 90;
  ledcWrite(steerServoPin, (270 - appliedSteer) * 900 / 270);

  // Serial.print("Applied speed: ");
  // Serial.print(appliedSpeed);
  // Serial.print(" Turn: ");
  // Serial.print(incomingMessage.turn);
  // Serial.print(" Set zero: " + String(incomingMessage.set_zero_steer));
  // Serial.print(" Applied Steering Angle: ");
  // Serial.println(appliedSteer);
  delay(5);
}

bool isReverse(int vel) {
  if (vel < 0) return true;
  return false;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data) {
  remoteLastRecvTime = millis();
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  digitalWrite(LED_BUILTIN, LOW);
  sendAck();
}

void sendAck() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&incomingMessage, sizeof(incomingMessage));

  digitalWrite(LED_BUILTIN, LOW);
  // Serial.print(millis());
  // Serial.print(" turn: "); Serial.println(incomingMessage.turn);
  // Serial.print("forward speed: "); Serial.println(incomingMessage.forwardSpeed);
  // Serial.println("_______________________");
}

float incrementToAngle(float steerAngle, float angleTarget, float increment) {
  if (steerAngle < angleTarget) {
    steerAngle = min(steerAngle + increment, angleTarget);
  } else {
    steerAngle = max(steerAngle - increment, angleTarget);
  }
  return steerAngle;
}

int sgn(int velocity) {
  if (velocity > 0) return 1;
  else if (velocity < 0) return -1;
  return 0;
}