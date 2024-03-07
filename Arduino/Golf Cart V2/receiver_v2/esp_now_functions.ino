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




void espNowSetup(){
  
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}


// config AP SSID
void configDeviceAP() {
  const char *SSID = "Golf_Cart_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  //print the mac address of sender
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Forward: "); Serial.print((int) incomingMessage.forward_speed);
  Serial.print(" Turn: "); Serial.print((int) incomingMessage.turn_speed);
  Serial.print(" BatteryVoltage: "); Serial.println((int) incomingMessage.remoteBatteryVoltage);
  digitalWrite(LED_BUILTIN, LOW);
}
