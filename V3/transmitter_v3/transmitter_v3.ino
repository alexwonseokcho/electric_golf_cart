#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//TO DO:
/*
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/gpio.html
Pins 0 to 5 can be used to wake it from deep sleep
- mac address hardcode
- pairing mode 
- optimize sleep states to save energy during operation 
- only send when there's a change in the state
- regular handshake to confirm connection & stop vehicle if disconnected 
- update voltage method
- maybe research other wifi modes of esp to increase range
- check sum on the receiver end
*/
/*

<< This Device Master >>
Flow: Master
Step 1 : ESPNow Init on Master and set it in STA mode
Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
Step 3 : Once found, add Slave as peer
Step 4 : Register for send callback
Step 5 : Start Transmitting data from Master to Slave

Flow: Slave
Step 1 : ESPNow Init on Slave
Step 2 : Update the SSID of Slave with a prefix of `slave`
Step 3 : Set Slave in AP mode
Step 4 : Register for receive callback and wait for data
Step 5 : Once data arrives, print it in the serial monitor

Note: Master and Slave have been defined to easily understand the setup.
Based on the ESPNOW API, there is no concept of Master and Slave.
Any devices can act as master or slave.
*/

#define UP_PIN 11
#define STOP_PIN 10
#define LEFT_PIN 12
#define RIGHT_PIN 8
#define DOWN_PIN 9
#define BAT_MONITOR_PIN 1

// #define UP_PIN D0
// #define STOP_PIN D1
// #define LEFT_PIN D2
// #define RIGHT_PIN D5
// #define DOWN_PIN D4


//deep sleep
// #define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
// gpio_num_t wakeup_pins[] = {GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12};
// RTC_DATA_ATTR int bootCount = 0;


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
// 74:4d:bd:81:ba:d4
// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Sent to: ");
  // Serial.println(macStr);
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    // Serial.println("ESPNow Init Success");
  } else {
    // Serial.println("ESPNow Init Failed");
    delay(500);
    ESP.restart();
  }
}

void sendData() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  Serial.print("steer angle: "); Serial.println(outgoingMessage.steer_angle);
  Serial.print("forward speed: "); Serial.println(outgoingMessage.forward_speed);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
  Serial.print(millis());
  Serial.print("\tSend Status: ");


  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }

  // neopixelWrite(LED_BUILTIN, 0, 10, 0);  // Off / black
  // RGB_BUILTIN
  // Serial.println(outgoingMessage.forward_speed);
  // Serial.println(outgoingMessage.steer_angle);
}

int lastEventTime;
int lastSentTime;
void setup() {
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);

  // pinMode(BAT_MONITOR_PIN, INPUT);

  Serial.begin(115200);

  //deep sleep wake
  // ++bootCount;
  // Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  // print_wakeup_reason();

  //If you were to use ext1, you would use it like
  // uint64_t bitmask = 0;
  // for(int i = 0; i < sizeof(wakeup_pins) / sizeof(wakeup_pins[0]); i++){
  //   bitmask = bitmask | BUTTON_PIN_BITMASK(wakeup_pins[i]);
  //   rtc_gpio_pullup_en(wakeup_pins[i]); 
  //   rtc_gpio_pulldown_dis(wakeup_pins[i]); 
  // }
  // esp_sleep_enable_ext1_wakeup_io(bitmask, ESP_EXT1_WAKEUP_ANY_LOW);

  
  
  //ESPNOW setup
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);


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
  
  lastEventTime = millis();
  lastSentTime = millis();
  
}

void batteryMonitor() {
  //with 1:1 voltage divider, 4.2V should be 2.1V --> 1023 * (2.1V / 3.3V) = 651
  //to go from ticks to V, 651 --> 4.2V, 0 --> 0V
  float batteryOffset = 290;  //calibrate later (detect ticks at 4.2V)
  float ticksToVoltage = 4.2 / batteryOffset;

  outgoingMessage.remoteBatteryVoltage = 0;  //analogRead(BAT_MONITOR_PIN) * ticksToVoltage;
}

int speedLevel = 0;  // -3 to 9 speed level, with [INSERT SOMETHING] being freespinning wheel (CODE THIS IN)
//first forward from standing start will increase it to level 4, with subsequent increments being 1
float steerAngle = 0;  // positive for right, negative for left
void loop() {
  bool changed = updateButtonPressState();

  //make sure it connects back when it disconnects

  batteryMonitor();

  // Serial.print("Forward: ");
  // Serial.print(outgoingMessage.forward_speed);
  // Serial.print("  Steer: ");
  // Serial.println( outgoingMessage.steer_angle);
  // Serial.print("  Battery V: ");
  // Serial.println(batteryVoltage);



  if (changed || millis() - lastSentTime > 500) {
    sendData();
    lastSentTime = millis();
  }
  if (millis() - lastEventTime > 60000 && outgoingMessage.forward_speed == 0) {
    enterDeepSleep();
  }

  // neopixelWrite(RGB_BUILTIN, 0, 0, 0);  // Red

  delay(5);
}

void enterDeepSleep() {
  //ENABLE DEEP SLEEP OF DW1000 CHIP TOO
  // digitalWrite(RGB_BUILTIN, LOW);
  // delay(1000);
  // digitalWrite(RGB_BUILTIN, HIGH);
  // delay(1000);

  // neopixelWrite(RGB_BUILTIN, 0, 0, 0);  // Off / black;
  // esp_deep_sleep_start();
}


bool pressed = false;
int lastSpeedLevel;
float lastSteerAngle;
float increment = 0.6;
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
    //hold this button down to enable free-spinning wheels
    // Serial.println("STOP");
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

bool isChangedSinceLast(int lastSpeed, int currentSpeed) {
  return !(lastSpeed == currentSpeed);
}


int sgn(float num){
  if(num > 0.0) 
    return 1;
  else if(num < 0.0)
    return -1;
  return 0;
}

// void print_wakeup_reason() {
//   esp_sleep_wakeup_cause_t wakeup_reason;

//   wakeup_reason = esp_sleep_get_wakeup_cause();

//   switch (wakeup_reason) {
//     case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
//     case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
//     case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
//     case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
//     case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
//     default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
//   }
// }

