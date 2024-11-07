#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
//nRF uses 7 8 11 12 13
#define UP_PIN 10
#define STOP_PIN 4 //done
#define LEFT_PIN 9
#define RIGHT_PIN A3
#define DOWN_PIN 2
#define BAT_MONITOR_PIN A6

#define LED_LOW_PIN 3
#define LED_MED_PIN 5
#define LED_HIGH_PIN 6

const byte address[6] = "00001";


void setup() {
  pinMode(UP_PIN, INPUT_PULLUP); pinMode(STOP_PIN, INPUT_PULLUP); pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP); pinMode(DOWN_PIN, INPUT_PULLUP);

  pinMode(BAT_MONITOR_PIN, INPUT);
  
  pinMode(LED_LOW_PIN, OUTPUT); pinMode(LED_MED_PIN, OUTPUT); pinMode(LED_HIGH_PIN, OUTPUT);

  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}


void speedLedHandler(int speedLevel){
  int positiveSpeedLevel = max(0, speedLevel);

  if(speedLevel >= 7){
    analogWrite(LED_LOW_PIN, 255);
    analogWrite(LED_MED_PIN, 255);
    analogWrite(LED_HIGH_PIN, (speedLevel - 6) * 80);
  }
  else if(speedLevel >= 4){
    analogWrite(LED_LOW_PIN, 255);
    analogWrite(LED_MED_PIN, (speedLevel - 3) * 80);
    analogWrite(LED_HIGH_PIN, 0);
  }
  else if(speedLevel >= 0){
    analogWrite(LED_LOW_PIN, speedLevel * 80);
    digitalWrite(LED_MED_PIN, 0); digitalWrite(LED_HIGH_PIN, 0);
    
  }
  else{
    digitalWrite(LED_LOW_PIN, (millis() % 500 < 250));
    digitalWrite(LED_MED_PIN, (millis() % 500 < 250)); 
    digitalWrite(LED_HIGH_PIN, (millis() % 500 < 250));
  }
}

float batteryMonitor(){
  //with 1:1 voltage divider, 4.2V should be 2.1V --> 1023 * (2.1V / 3.3V) = 651 
  //to go from ticks to V, 651 --> 4.2V, 0 --> 0V
  float batteryOffset = 651; //calibrate later (detect ticks at 4.2V)
  float ticksToVoltage = 4.2/batteryOffset;

  return analogRead(BAT_MONITOR_PIN) * ticksToVoltage;
}

int speedLevel = 0; // -3 to 9 speed level, with [INSERT SOMETHING] being freespinning wheel (CODE THIS IN)
//first forward from standing start will increase it to level 4, with subsequent increments being 1
int turnSpeed = 0; // positive for right, negative for left
void loop() {
  bool changed = remoteSpeedHandler();
  speedLedHandler(speedLevel);
  
  // 600 max? thus actual speed will be 9 * 65 = 585 RPM ~~ 20 kph
  // turn speed can be like 100?
  float batteryVoltage = batteryMonitor();
  int forwardSpeed = speedLevel * 20;
  int steerSpeed = turnSpeed /10;
//
  Serial.print("Forward: ");
  Serial.print(forwardSpeed);
  Serial.print("  Steer: ");
  Serial.print(steerSpeed);
  Serial.print("  Battery V: ");
  Serial.println(batteryVoltage);

  
  
  int message[4] = {forwardSpeed, steerSpeed, batteryVoltage * 100, 0};

  if(changed || millis() % 250 < 30) radio.write(&message, sizeof(message));
  //change later -- I think changed logic doesn't wokr properly
  //send update every 250 ms as well to ensure connection
}


bool pressed = false;
bool remoteSpeedHandler() {
  bool stop_pin = !digitalRead(STOP_PIN);
  bool up = !digitalRead(UP_PIN);
  bool down = !digitalRead(DOWN_PIN);
  bool left = !digitalRead(LEFT_PIN);
  bool right = !digitalRead(RIGHT_PIN);
  bool changed = false;

  if(stop_pin && !pressed){
    pressed = true;
    speedLevel = 0;
    changed = true;
    //hold this button down to enable free-spinning wheels
    Serial.println("STOP");
  }
  else if(up && !pressed){
    pressed = true;
    if(speedLevel == 0) speedLevel = 4;
    else  speedLevel = min(9, speedLevel + 1);
    changed = true;
  }
  else if(down && !pressed){
    pressed = true; changed = true;
    speedLevel = max(-3, speedLevel - 1);
  }
  if(left){ //turning should stop as soon as button is released
    changed = true;
    if(turnSpeed == 0) turnSpeed = -400;
    else turnSpeed = max(turnSpeed - 3, -1500);
  }
  else if(right){
    changed = true;
    if(turnSpeed == 0) turnSpeed = 400;
    else turnSpeed = min(turnSpeed + 3, 1500);
  }
  else{
    changed = true;
    turnSpeed = 0;
  }

  if (!stop_pin && !up && !down)   {
    pressed = false;
    if(!left && !right) changed = false;
//    Serial.println("RESET");
  }

//  Serial.println(pressed);
//  Serial.println(changed);
  return changed;
}
