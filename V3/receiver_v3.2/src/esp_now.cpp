#include "esp_now.hpp"
#include "pins.hpp"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

uint8_t espNowBroadcastAddress[] = { 0x84, 0xfc, 0xe6, 0x6b, 0x87, 0x08 };
esp_now_peer_info_t peerInfo;

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

unsigned long remoteLastRecvTime = millis();
incoming_message incomingMessage = {0, 0, 0, false};

void sendAck() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&incomingMessage, sizeof(incomingMessage));

  digitalWrite(LED_BUILTIN, LOW);
  // Serial.print(millis());
  // Serial.print(" turn: "); Serial.println(incomingMessage.turn);
  // Serial.print("forward speed: "); Serial.println(incomingMessage.forwardSpeed);
  // Serial.println("_______________________");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data) {
  remoteLastRecvTime = millis();
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  digitalWrite(LED_BUILTIN, LOW);
  sendAck();
}



void espNowSetup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    delay(2000);
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    delay(2000);
    ESP.restart();
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

}

