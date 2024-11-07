#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_util.h>
#include <shtp.h>

#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include "defines.h"

HardwareSerial HoverSerial(1);

typedef struct incoming_message{
  int16_t forward_speed;
  int16_t turn_speed;
  int16_t remoteBatteryVoltage;
} incoming_message;

typedef struct outgoing_message{
  int16_t cartBatteryVoltage;
} outgoing_message;

incoming_message incomingMessage;
outgoing_message outgoingMessage;

// #define LED_BUILTIN 21 //flashes every 2 seconds

int remoteLastRecvTime;

void setup() 
{
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, D7, D6);

  pinMode(LED_BUILTIN, OUTPUT);
  // initScreen();
  espNowSetup();
  // setupIMU();
  remoteLastRecvTime = millis();
}


float targetHeading = 0;

unsigned long nextSendTime = 0;
int lastForward;
int forward = 0;
void loop(void)
{ 
  unsigned long timeNow = millis();
  // getIMUData();

  // Check for new received data
  ReceiveHoverboard();

  //make sure if the remote disconnects, the cart stops right away

  // int turnPower = headingPID();
  // Serial.println("Turn power: " + (String) turnPower);

  forward = incomingMessage.forward_speed;
  int turnPower = incomingMessage.turn_speed;

  if(millis() - remoteLastRecvTime > 2000){
    forward = 0;
    turnPower = 0;
  }
  //Low pass filter for accelerating forward
  if(forward > lastForward) 
    forward = min(lastForward + 1, forward);

  lastForward = forward;

  // Send commands
  if (nextSendTime < timeNow){
    // Serial.println("turnpower: " + (String) turnPower + "forward: " + (String) forward);
    SendHoverboard(turnPower, forward);
    nextSendTime = timeNow + TIME_SEND;
  }



  Serial.println("forward: " + (String) forward + " turn: " + (String) incomingMessage.turn_speed);

  
  // processAnchorDistance();
  // calculatePosition();
  // displayInfo();

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%1000)<1000);
  yield();
}
