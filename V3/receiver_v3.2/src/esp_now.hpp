#ifndef _ESP_NOW_HPP_
#define _ESP_NOW_HPP_

#include <Arduino.h>

typedef struct incoming_message {
  int16_t forwardSpeed;
  int8_t turn;
  int16_t remoteBatteryVoltage;
  bool set_zero_steer;
} incoming_message;

extern incoming_message incomingMessage;
extern unsigned long remoteLastRecvTime;
void espNowSetup();


#endif