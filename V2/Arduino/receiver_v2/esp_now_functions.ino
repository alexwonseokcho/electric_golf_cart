/*
<< This Device Slave >>

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
Any devices can act as master or salve.
*/

typedef struct anchorMessage {
  int id;
  float distance;
}anchorMessage;

// Create a structure to hold the readings from each board
anchorMessage incomingAnchorMessage;

typedef struct anchorDistances {
  float anc1;
  float anc2;
  float anc3;
}anchorDistances;

anchorDistances ancReading;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if(data_len == 6){ //this is a remote controller packet
    remoteLastRecvTime = millis();
    memcpy(&incomingMessage, data, sizeof(incomingMessage));
    // Serial.println("")
  }
  else if(data_len == 8){ //this is a anchor packet
    memcpy(&incomingAnchorMessage, data, sizeof(incomingAnchorMessage));
    if(incomingAnchorMessage.id == 1){
      ancReading.anc1 = incomingAnchorMessage.distance;
    }
    else if(incomingAnchorMessage.id == 2){
      ancReading.anc2 = incomingAnchorMessage.distance;
    }
    else if(incomingAnchorMessage.id == 3){
      ancReading.anc3 = incomingAnchorMessage.distance;
    }
    // Serial.printf("anc1: %f \t", ancReading.anc1);
    // Serial.printf("anc2: %f \t", ancReading.anc2);
    // Serial.printf("anc3: %f \n", ancReading.anc3);
    // Serial.println();
    Serial.println(incomingMessage.forward_speed);

    Serial.println(incomingMessage.turn_speed);
  }



  //print the mac address of sender
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  
  // Serial.print("Forward: "); Serial.print((int) incomingMessage.forward_speed);
  // Serial.print(" Turn: "); Serial.print((int) incomingMessage.turn_speed);
  // Serial.print(" BatteryVoltage: "); Serial.println((int) incomingMessage.remoteBatteryVoltage);
  digitalWrite(LED_BUILTIN, LOW);
}


void espNowSetup(){
  
  //Set device in AP mode to begin with
  // WiFi.mode(WIFI_AP);
  // // configure device AP mode
  // configDeviceAP();
  // // This is the mac address of the Slave in AP Mode
  // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // // Init ESPNow with a fallback logic
  // InitESPNow();
  // // Once ESPNow is successfully Init, we will register for recv CB to
  // // get recv packer info.
  // esp_now_register_recv_cb(OnDataRecv);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

// // Init ESP Now with fallback
// void InitESPNow() {
//   WiFi.disconnect();
//   if (esp_now_init() == ESP_OK) {
//     Serial.println("ESPNow Init Success");
//   }
//   else {
//     Serial.println("ESPNow Init Failed");
//     ESP.restart();
//   }
// }


// // config AP SSID
// void configDeviceAP() {
//   const char *SSID = "Golf_Cart_1";
//   bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
//   if (!result) {
//     Serial.println("AP Config failed.");
//   } else {
//     Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
//     Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
//   }
// }

