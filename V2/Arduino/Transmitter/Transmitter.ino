// https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/gpio.html
// Pins 0 to 5 can be used to wake it from deep sleep
#include <esp_now.h>
#include <WiFi.h>

//TO DO: 
/*
- mac address hardcode
- pairing mode 
- optimize sleep states to save energy during operation 
- only send when there's a change in the state
- regular handshake to confirm connection & stop vehicle if disconnected 
- update voltage method
- maybe research other wifi modes of esp to increase range
*/

//CHANGE THESE 
#define UP_BUT 1
#define LEFT_BUT 2
#define STOP_BUT 3
#define RIGHT_BUT 4
#define DOWN_BUT 5

//REPLACE THIS WITH MAC ADDRESS & ALSO MAKE A PAIRING MODE
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct transmitter_message{
  float pressedButton; //button that is being pressed currently
  //    0
  // 1  2  3
  //    4
  float remoteBatteryVoltage;
} transmitter_message;

typedef struct incoming_message{
  float cartBatteryVoltage
} incoming_message;

transmitter_message outgoingMessage;
transmitter_message incomingMessage;

void loop()
{
  //make it so if more than one button is pressed, give priority to:
  //stopping, turning, and forward/backward

  updateButtonPressState();
  updateBatteryVoltage();


  //optimize with sleep states during operation later 

  //have a handshake every couple seconds to confirm connection
  


}

// Callback when data is sent
// on the receiver, count up with a status_count
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)  success = "Packet Delivered";
  else  success = "Packet Failed to Deliver";
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println(incomingReadings.cartBatteryVoltage);
}

void updateButtonPressState(){
  if(digitalRead(STOP_BUT) == 0)) outgoingMessage.pressedButton = 2;
  else if(digitalRead(LEFT_BUT) == 0)) outgoingMessage.pressedButton = 1;
  else if(digitalRead(RIGHT_BUT) == 0)) outgoingMessage.pressedButton = 3;
  else if(digitalRead(UP_BUT) == 0)) outgoingMessage.pressedButton = 0;
  else if(digitalRead(DOWN_BUT) == 0)) outgoingMessage.pressedButton = 4;
  else outgoingMessage.pressedButton = 0;
}

void updateBatteryVoltage(){
  //update voltage here
  outgoingMessage.remoteBatteryVoltage = 3.7;
}

void setup()
{
  pinMode(UP_BUT, INPUT_PULLUP);
  pinMode(LEFT_BUT, INPUT_PULLUP);
  pinMode(STOP_BUT, INPUT_PULLUP);
  pinMode(RIGHT_BUT, INPUT_PULLUP);
  pinMode(DOWN_BUT, INPUT_PULLUP);
  Serial.begin(115200);

  //maybe research other wifi modes to increase range
  WiFi.mode(WIFI_STA)

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  
  //IMPLEMENT DEEPSLEEP 
  // //The following command is intended to turn off all RTC peripherals in deep sleep.
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  // esp_deep_sleep_start();
}