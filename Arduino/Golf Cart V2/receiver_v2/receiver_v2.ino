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

void setup() 
{
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 2, 3);

  pinMode(LED_BUILTIN, OUTPUT);
  initScreen();
  espNowSetup();
}

unsigned long nextSendTime = 0;
int lastForward;
int forward;
void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  ReceiveHoverboard();

  //make sure if the remote disconnects, the cart stops right away

  // Send commands
  if (nextSendTime > timeNow) return;
  
  forward = incomingMessage.forward_speed;
  //Low pass filter for accelerating forward
  if(forward > lastForward) 
    forward = min(lastForward + 1, forward);

  lastForward = forward;
  nextSendTime = timeNow + TIME_SEND;
  SendHoverboard(incomingMessage.turn_speed, forward);

  Serial.println(forward);
  Serial.println(incomingMessage.turn_speed);

  displayInfo();

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%1000)<1000);
}
