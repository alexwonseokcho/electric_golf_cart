
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "driver/rtc_io.h"

//TO DO:
/*
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/gpio.html
Pins 0 to 5 can be used to wake it from deep sleep
- mac address hardcode
- pairing mode 
- optimize sleep states to save energy during operation 
- update voltage method

*/

#define UP_PIN D0
#define STOP_PIN D1
#define LEFT_PIN D2
#define RIGHT_PIN D5
#define DOWN_PIN D4
// #define BAT_MONITOR_PIN A2

//deep sleep
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
gpio_num_t wakeup_pins[] = {gpio_num_t(UP_PIN), gpio_num_t(STOP_PIN), gpio_num_t(LEFT_PIN), gpio_num_t(RIGHT_PIN), gpio_num_t(DOWN_PIN)};
RTC_DATA_ATTR int bootCount = 0;

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Variable to store if sending data was successful
float cartBatteryVoltage;

typedef struct transmitter_message {
  int16_t forward_speed;
  int16_t steer_angle;
  int16_t remoteBatteryVoltage;
} transmitter_message;

typedef struct incoming_message {
  int16_t cartBatteryVoltage;
} incoming_message;

transmitter_message outgoingMessage;
incoming_message incomingMessage;

//ESP NOW
uint8_t espNowBroadcastAddress[] = { 0x74, 0x4d, 0xbd, 0x81, 0xba, 0xd4 };

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  // Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  cartBatteryVoltage = incomingMessage.cartBatteryVoltage;
}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
  } else {
    delay(500);
    ESP.restart();
  }
}

void sendData() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  // Serial.print("Sending: "); Serial.println(outgoingMessage);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
  // Serial.print(millis());
  // Serial.print("\t");
  // Serial.print("Send Status: ");
  digitalWrite(LED_BUILTIN, LOW);
    Serial.print("steer angle: "); Serial.println(outgoingMessage.steer_angle);
  Serial.print("forward speed: "); Serial.println(outgoingMessage.forward_speed);
  Serial.print(millis());
  Serial.print("\tSend Status: ");

  // Serial.println(outgoingMessage.forward_speed);
  // Serial.println(outgoingMessage.turn_speed);
  if (result == ESP_OK) {
    // Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    // Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    // Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    // Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    // Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    // Serial.println("Peer not found.");
  } else {
    // Serial.println("Not sure what happened");
  } 
  
}
int lastEventTime; int lastSentTime;
void setup() {
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(BAT_MONITOR_PIN, INPUT);

  Serial.begin(115200);


  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
    ESP.restart();
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  print_wakeup_reason();


  //deep sleep wake
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  //If you were to use ext1, you would use it like
  uint64_t bitmask = 0;
  for(int i = 0; i < sizeof(wakeup_pins) / sizeof(wakeup_pins[0]); i++){
    bitmask = bitmask | BUTTON_PIN_BITMASK(wakeup_pins[i]);
    rtc_gpio_pullup_en(wakeup_pins[i]); 
    rtc_gpio_pulldown_dis(wakeup_pins[i]); 
  }
  esp_sleep_enable_ext1_wakeup_io(bitmask, ESP_EXT1_WAKEUP_ANY_LOW);


  lastEventTime = millis();
  lastSentTime = millis();
  // bool isPaired = false;
  // do{
  //   ScanForSlave();
  //   // If Slave is found, it would be populate in `slave` variable
  //   // We will check if `slave` is defined and then we proceed further
  //   if(slave.channel == CHANNEL){
  //     isPaired = manageSlave();
  //   }
  //   Serial.println("Searching...");
  //   delay(50);
  // }while(!isPaired);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  // switch (wakeup_reason) {
  //   case ESP_SLEEP_WAKEUP_EXT0:
  //     Serial.println("Wakeup caused by external signal using RTC_IO");
  //     break;
  //   case ESP_SLEEP_WAKEUP_EXT1:
  //     Serial.println("Wakeup caused by external signal using RTC_CNTL");
  //     break;
  //   case ESP_SLEEP_WAKEUP_TIMER:
  //     Serial.println("Wakeup caused by timer");
  //     break;
  //   case ESP_SLEEP_WAKEUP_TOUCHPAD:
  //     Serial.println("Wakeup caused by touchpad");
  //     break;
  //   case ESP_SLEEP_WAKEUP_ULP:
  //     Serial.println("Wakeup caused by ULP program");
  //     break;
  //   default:
  //     Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
  //     break;
  // }
}

void batteryMonitor() {
  //with 1:1 voltage divider, 4.2V should be 2.1V --> 1023 * (2.1V / 3.3V) = 651
  //to go from ticks to V, 651 --> 4.2V, 0 --> 0V
  float batteryOffset = 290;  //calibrate later (detect ticks at 4.2V)
  float ticksToVoltage = 4.2 / batteryOffset;

  outgoingMessage.remoteBatteryVoltage = 0; //analogRead(BAT_MONITOR_PIN) * ticksToVoltage;
}

int speedLevel = 0;  // -3 to 9 speed level, with [INSERT SOMETHING] being freespinning wheel (CODE THIS IN)
//first forward from standing start will increase it to level 4, with subsequent increments being 1
float steerAngle = 0;  // positive for right, negative for left
void loop() {
  bool changed = updateButtonPressState();

  //make sure it connects back when it disconnects

  // 600 max? thus actual speed will be 9 * 65 = 585 RPM ~~ 20 kph
  // turn speed can be like 100?
  batteryMonitor();

  // Serial.print("Forward: ");
  // Serial.print(forwardSpeed);
  // Serial.print("  Steer: ");
  // Serial.println(steerSpeed);
  // Serial.print("  Battery V: ");
  // Serial.println(batteryVoltage);

  if(changed || millis() - lastSentTime > 500) {
    sendData();
    lastSentTime = millis();
  }
  if(millis() - lastEventTime > 10000 && outgoingMessage.forward_speed == 0 && outgoingMessage.steer_angle == 0){
    enterDeepSleep();
    // Serial.println("Going to sleep...");
  }
  
  //change later -- I think changed logic doesn't wokr properly
  //send update every 250 ms as well to ensure connection - heartbeat
  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
}

void enterDeepSleep() {
  //ENABLE DEEP SLEEP OF DW1000 CHIP TOO
  for(int i = 0; i < 5; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }

  esp_deep_sleep_start();
}


bool pressed = false;
int lastSpeedLevel;
float lastSteerAngle;
float increment = 0.2;
bool updateButtonPressState() {
  bool stop_pin = !digitalRead(STOP_PIN);
  bool up = !digitalRead(UP_PIN);
  bool down = !digitalRead(DOWN_PIN);
  bool left = !digitalRead(LEFT_PIN);
  bool right = !digitalRead(RIGHT_PIN);
  bool changed = false;

  if (stop_pin && !pressed) {
    pressed = true;
    speedLevel = 0;
    steerAngle = 0;
  } else if (up && !pressed) {
    pressed = true; 
    if (speedLevel == 0) speedLevel = 4;
    else speedLevel = min(50, speedLevel + 1);
  } else if (down && !pressed) {
    pressed = true;
    speedLevel = max(-3, speedLevel - 1);
  }
  if (left) {  //turning should stop as soon as button is released
    steerAngle = max(steerAngle - increment, float(-135.0));
  } else if (right) {
    steerAngle = min(steerAngle + increment, float(135.0));
  }
  else{
    steerAngle = max(float(0.0), fabs(steerAngle) - increment) * sgn(steerAngle);
  }

  if (!stop_pin && !up && !down) {
    pressed = false;
    if (!left && !right) changed = false;
    //    Serial.println("RESET");
  }

  if (stop_pin && up && down && left && right){ //user can restart like this
    ESP.restart();
  }


  changed = !(lastSteerAngle == steerAngle) || !(lastSpeedLevel == speedLevel);
  if (changed) {
    lastEventTime = millis();
  }
  lastSpeedLevel = speedLevel;
  lastSteerAngle = steerAngle;

  outgoingMessage.forward_speed = speedLevel * 4;
  outgoingMessage.steer_angle = int(steerAngle);

  return changed;
}

bool isChangedSinceLast(int lastSpeed, int currentSpeed){
  return !(lastSpeed == currentSpeed);
}

int sgn(float num){
  if(num > 0.0) 
    return 1;
  else if(num < 0.0)
    return -1;
  return 0;
}