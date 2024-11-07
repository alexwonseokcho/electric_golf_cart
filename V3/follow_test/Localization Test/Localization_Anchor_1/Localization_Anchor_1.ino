

/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorMain_TWR.ino
 * 
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

// #include <HardwareSerial.h>

// HardwareSerial SendSerial(1);



#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include <esp_now.h>
#include <WiFi.h>


typedef struct Position {
    double x;
    double y;
} Position;

const uint8_t PIN_RST = 20; // reset pin
const uint8_t PIN_IRQ = 21; // irq pin
const uint8_t PIN_SS = 7; // spi select pin

const uint8_t PIN_MOSI = 10;
const uint8_t PIN_MISO = 9;
const uint8_t PIN_CLK = 8;

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

Position position_self = {0,0};
Position position_B = {3,0};
Position position_C = {3,2.5};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

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
    true /* This allows blink frames */
};

//ESP NOW
uint8_t espNowBroadcastAddress[] = {0x34, 0x85, 0x18, 0xac, 0x08, 0x28};

typedef struct anchorDistance {
  int id = 1; //anchor 1
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
    delay(50);
    Serial.begin(9600);
    Serial.println("dlskjflsdf'"); 
    Serial.println("dlskjflsdf'"); 
    Serial.println("dlskjflsdf'"); 
    Serial.println("dlskjflsdf'"); 
    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));

    // SendSerial.begin(9600, SERIAL_8N1, -1, D3);

    
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_SS);

    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(64);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(1);
	
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


/* using https://math.stackexchange.com/questions/884807/find-x-location-using-3-known-x-y-location-using-trilateration */
void calculatePosition(double &x, double &y) {

    /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*position_self.x) + (2*position_B.x) );
    double B = ( (-2*position_self.y) + (2*position_B.y) );
    double C = (range_self*range_self) - (range_B*range_B) - (position_self.x*position_self.x) + (position_B.x*position_B.x) - (position_self.y*position_self.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}


void loop() {

  // Serial.println("dlskjflsdf'"); 
  // Serial.println("dlskjflsdf'"); 
  // Serial.println("dlskjflsdf'"); 
  // Serial.println("dlskjflsdf'"); 
  if(DW1000NgRTLS::receiveFrame()){
    size_t recv_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[recv_len];
    DW1000Ng::getReceivedData(recv_data, recv_len);


    if(recv_data[0] == BLINK) {
        DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
        DW1000NgRTLS::waitForTransmission();

        RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
        if(result.success){
          
          anchorDistancePacket.distance = result.range;

          // Send message via ESP-NOW
          esp_err_t espNowResult = esp_now_send(espNowBroadcastAddress, (uint8_t *) &anchorDistancePacket, sizeof(anchorDistancePacket));
          // Serial.println(anchorDistancePacket.distance);
          
          String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
          rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
          Serial.println(rangeString);
      }

    }
  }

  

  // Send message via ESP-NOW
  esp_err_t espNowResult = esp_now_send(espNowBroadcastAddress, (uint8_t *) &anchorDistancePacket, sizeof(anchorDistancePacket));
  Serial.println(anchorDistancePacket.distance);
  
  yield();
}