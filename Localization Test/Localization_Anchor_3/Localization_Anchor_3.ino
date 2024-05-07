/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorC_TWR.ino
 * 
 * This is an example slave anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

const uint8_t PIN_RST = 5; // reset pin
const uint8_t PIN_IRQ = 6; // irq pin
const uint8_t PIN_SS = 1; // spi select pin

const uint8_t PIN_MOSI = 4;
const uint8_t PIN_MISO = 3;
const uint8_t PIN_CLK = 2;

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:03"; 

byte main_anchor_address[] = {0x01, 0x00};

uint16_t blink_rate = 200;



device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    true, //was false - NLOS setting - maybe set the anchors to true too 
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_110KBPS,//was 850
    PulseFrequency::FREQ_64MHZ, //was 16
    PreambleLength::LEN_2048, //was 256
    PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};


//ESP NOW
uint8_t espNowBroadcastAddress[] = {0x34, 0x85, 0x18, 0xac, 0x08, 0x28};

typedef struct anchorDistance {
  int id = 3; //anchor 3
  float distance;
} anchorDistance;

// Create a struct_message called myData
anchorDistance anchorDistancePacket;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### arduino-DW1000Ng-ranging-anchor-C ###"));

  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_SS);
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
  
  DW1000Ng::setEUI(EUI);

  DW1000Ng::setPreambleDetectionTimeout(64);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

  DW1000Ng::setNetworkId(RTLS_APP_ID);
  DW1000Ng::setDeviceAddress(3);

  DW1000Ng::setAntennaDelay(16436);
  
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);

  //ESPNOW setup
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

 
void loop() {
  RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::ACTIVITY_FINISHED, blink_rate);
  if(result.success) {
    delay(4); // Tweak based on your hardware

    //sometimes, power shoots to inf and distance becomes 0 -> filter this
    anchorDistancePacket.distance = result.range;

    // Send message via ESP-NOW
    esp_err_t espNowResult = esp_now_send(espNowBroadcastAddress, (uint8_t *) &anchorDistancePacket, sizeof(anchorDistancePacket));
    // Serial.println(anchorDistancePacket.distance);

    // String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
    // rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
    // Serial.println(rangeString);
  }
  
  yield();
}