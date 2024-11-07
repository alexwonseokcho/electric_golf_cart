
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

#define UP_PIN D0
#define STOP_PIN D1
#define LEFT_PIN D2
#define RIGHT_PIN D5
#define DOWN_PIN D4

// #define BAT_MONITOR_PIN A2


#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Variable to store if sending data was successful
float cartBatteryVoltage;

typedef struct transmitter_message {
  int16_t forward_speed;
  int16_t turn_speed;
  int16_t remoteBatteryVoltage;
} transmitter_message;

typedef struct incoming_message {
  int16_t cartBatteryVoltage;
} incoming_message;

transmitter_message outgoingMessage;
incoming_message incomingMessage;

//ESP NOW
uint8_t espNowBroadcastAddress[] = { 0x34, 0x85, 0x18, 0xac, 0x08, 0x28 };
// dc:54:75:df:1d:0c
// 34:85:18:ac:08:28
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
    // Serial.println("ESPNow Init Success");
  } else {
    // Serial.println("ESPNow Init Failed");
    delay(500);
    ESP.restart();
  }
}


// // Scan for slaves in AP mode
// void ScanForSlave() {
//   int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
//   // reset on each scan
//   bool slaveFound = 0;
//   memset(&slave, 0, sizeof(slave));

//   Serial.println("");
//   if (scanResults == 0) {
//     Serial.println("No WiFi devices in AP Mode found");
//   } else {
//     Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
//     for (int i = 0; i < scanResults; ++i) {
//       // Print SSID and RSSI for each device found
//       String SSID = WiFi.SSID(i);
//       int32_t RSSI = WiFi.RSSI(i);
//       String BSSIDstr = WiFi.BSSIDstr(i);

//       if (PRINTSCANRESULTS) {
//         Serial.print(i + 1);
//         Serial.print(": ");
//         Serial.print(SSID);
//         Serial.print(" (");
//         Serial.print(RSSI);
//         Serial.print(")");
//         Serial.println("");
//       }
//       delay(10);
//       // Check if the current device starts with `Slave`
//       if (SSID.indexOf("Golf_Cart") == 0) {
//         // SSID of interest
//         Serial.println("Found a Slave.");
//         Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
//         // Get BSSID => Mac Address of the Slave
//         int mac[6];
//         if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
//           for (int ii = 0; ii < 6; ++ii ) {
//             slave.peer_addr[ii] = (uint8_t) mac[ii];
//           }
//         }

//         slave.channel = CHANNEL; // pick a channel
//         slave.encrypt = 0; // no encryption

//         slaveFound = 1;
//         // we are planning to have only one slave in this example;
//         // Hence, break after we find one, to be a bit efficient
//         break;
//       }
//     }
//   }

//   if (slaveFound) {
//     Serial.println("Slave Found, processing..");
//   } else {
//     Serial.println("Slave Not Found, trying again.");
//   }

//   // clean up ram
//   WiFi.scanDelete();
// }


// // Check if the slave is already paired with the master.
// // If not, pair the slave with master
// bool manageSlave() {
//   if (slave.channel == CHANNEL) {
//     if (DELETEBEFOREPAIR) {
//       deletePeer();
//     }

//     Serial.print("Slave Status: ");
//     // check if the peer exists
//     bool exists = esp_now_is_peer_exist(slave.peer_addr);
//     if (exists) {
//       // Slave already paired.
//       Serial.println("Already Paired");
//       return true;
//     } else {
//       // Slave not paired, attempt pair
//       esp_err_t addStatus = esp_now_add_peer(&slave);
//       if (addStatus == ESP_OK) {
//         // Pair success
//         Serial.println("Pair success");
//         return true;
//       } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//         // How did we get so far!!
//         Serial.println("ESPNOW Not Init");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
//         Serial.println("Invalid Argument");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
//         Serial.println("Peer list full");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
//         Serial.println("Out of memory");
//         return false;
//       } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
//         Serial.println("Peer Exists");
//         return true;
//       } else {
//         Serial.println("Not sure what happened");
//         return false;
//       }
//     }
//   } else {
//     // No slave found to process
//     Serial.println("No Slave found to process");
//     return false;
//   }
// }

// void deletePeer() {
//   esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
//   Serial.print("Slave Delete Status: ");
//   if (delStatus == ESP_OK) {
//     // Delete success
//     Serial.println("Success");
//   } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//     // How did we get so far!!
//     Serial.println("ESPNOW Not Init");
//   } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
//     Serial.println("Invalid Argument");
//   } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
//     Serial.println("Peer not found.");
//   } else {
//     Serial.println("Not sure what happened");
//   }
// }

void sendData() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  // Serial.print("Sending: "); Serial.println(outgoingMessage);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
  Serial.print(millis());
  Serial.print("\t");
  Serial.print("Send Status: ");

  Serial.println(outgoingMessage.forward_speed);
  Serial.println(outgoingMessage.turn_speed);
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

  // pinMode(BAT_MONITOR_PIN, INPUT);

  Serial.begin(115200);

  // WiFi.mode(WIFI_STA);
  // esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Serial.println("ESPNow/Basic/Master Example");
  // // This is the mac address of the Master in Station Mode
  // Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // // Init ESPNow with a fallback logic
  // InitESPNow();
  // // Once ESPNow is successfully Init, we will register for Send CB to
  // // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  // esp_now_register_recv_cb(OnDataRecv);
  //ESPNOW setup
  // Set device as a Wi-Fi Station
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
  // print_wakeup_reason();

 
  // esp_deep_sleep_enable_gpio_wakeup(1 << STOP_PIN | 1 << UP_PIN,
  //                                   ESP_GPIO_WAKEUP_GPIO_LOW);

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
int turnSpeed = 0;  // positive for right, negative for left
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

  // if(millis() - lastEventTime > 60000 && outgoingMessage.forward_speed == 0 && outgoingMessage.turn_speed == 0){
  //   enterDeepSleep();
  //   // Serial.println("Going to sleep...");
  // }
  
  //change later -- I think changed logic doesn't wokr properly
  //send update every 250 ms as well to ensure connection - heartbeat
  
  delay(5);
}

// void enterDeepSleep(){
//   //ENABLE DEEP SLEEP OF DW1000 CHIP TOO
//   // Serial.println("Going to sleep now");

//   digitalWrite((gpio_num_t) STOP_PIN, HIGH);
//   // gpio_hold_en((gpio_num_t) STOP_PIN);
//   digitalWrite((gpio_num_t) UP_PIN, HIGH);
//   // gpio_hold_en((gpio_num_t) UP_PIN);
//   esp_deep_sleep_start();
// }


bool pressed = false;
int lastSpeedLevel;
int lastTurnLevel;
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
    if (turnSpeed == 0) turnSpeed = -400;
    else turnSpeed = max(turnSpeed - 3, -1500);
  } else if (right) {
    if (turnSpeed == 0) turnSpeed = 400;
    else turnSpeed = min(turnSpeed + 3, 1500);
  } else {
    turnSpeed = 0;
  }

  if (!stop_pin && !up && !down) {
    pressed = false;
    if (!left && !right) changed = false;
    //    Serial.println("RESET");
  }


  changed = !(lastTurnLevel == turnSpeed) || !(lastSpeedLevel == speedLevel);
  if(changed){
    lastEventTime = millis();
  }
  lastSpeedLevel = speedLevel;
  lastTurnLevel = turnSpeed;

  outgoingMessage.forward_speed = speedLevel * 25;
  outgoingMessage.turn_speed = turnSpeed / 10;

  return changed;
}

bool isChangedSinceLast(int lastSpeed, int currentSpeed){
  return !(lastSpeed == currentSpeed);
}