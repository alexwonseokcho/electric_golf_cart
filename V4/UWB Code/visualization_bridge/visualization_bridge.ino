#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

typedef struct anchor_packet {
  uint8_t id; //anchor 1 is left (main anchor), 2 is right
  float distance;
} anchorDistance;

float leftDistance = 0;
float rightDistance = 0;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // Serial.println("RECEIVED");
    if(data[0] == 1){ //anchor 1 = left
        anchor_packet anchorData;
        memcpy(&anchorData, data, sizeof(anchor_packet));
        leftDistance = anchorData.distance;
    }
    else if(data[0] == 2){ //anchor 2 = right
        anchor_packet anchorData;
        memcpy(&anchorData, data, sizeof(anchor_packet));
        rightDistance = anchorData.distance;
    }
}


void setup(){
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        ESP.restart();
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop(){
    Serial.println(String(leftDistance) + " " + String(rightDistance));
}