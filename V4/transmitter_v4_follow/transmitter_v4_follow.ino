#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "driver/rtc_io.h"



#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>



//TO DO:
/*
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/gpio.html
Pins 0 to 5 can be used to wake it from deep sleep
- mac address hardcode
- pairing mode 
- update voltage method
*/

#define UP_PIN D0
#define STOP_PIN D1
#define LEFT_PIN D2
#define RIGHT_PIN D5
#define DOWN_PIN D4
// #define BAT_MONITOR_PIN A2

const uint8_t PIN_RST = D6;
const uint8_t PIN_SS = D7;
const uint8_t PIN_MOSI = D10;
const uint8_t PIN_MISO = D9;
const uint8_t PIN_CLK = D8;

//###############follow me setup
// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:00";

volatile uint32_t blink_rate = 50;

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

sleep_configuration_t SLEEP_CONFIG = {
    false,  // onWakeUpRunADC   reg 0x2C:00
    false,  // onWakeUpReceive
    false,  // onWakeUpLoadEUI
    true,   // onWakeUpLoadL64Param
    true,   // preserveSleep
    true,   // enableSLP    reg 0x2C:06
    false,  // enableWakePIN
    true    // enableWakeSPI
};

bool update_pos = false;
//#############

//deep sleep
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
gpio_num_t wakeup_pins[] = {gpio_num_t(UP_PIN), gpio_num_t(STOP_PIN), gpio_num_t(LEFT_PIN), gpio_num_t(RIGHT_PIN), gpio_num_t(DOWN_PIN)};

// Variable to store if sending data was successful
float cartBatteryVoltage;

typedef struct packet {
  int16_t forward_speed;
  int8_t turn;
  int16_t remoteBatteryVoltage;
  bool set_zero_steer;
} transmitter_message;

RTC_DATA_ATTR packet cur_state = {0, 0, 0};
RTC_DATA_ATTR packet last_state = {0, 0, 0};

bool ack = false; //command send acknowledge
int lastEventTime; int lastSentTime;

//ESP NOW
uint8_t espNowBroadcastAddress[] = { 0x80, 0x65, 0x99, 0x88, 0x2d, 0x68 };
// 80:65:99:88:2d:68
// dc:54:75:df:1d:0c
// Create peer interface
esp_now_peer_info_t peerInfo; 

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
}

typedef struct tagPositionStruct{
  float left_distance;
  float right_distance;
  double r = 0;
  double theta = 0;
} tagPositionStruct;

tagPositionStruct tagPosition;

//used only for esp now
typedef struct anchorDistance {
  int id; //anchor 1 is left (main anchor), 2 is right
  float distance;
} anchorDistance;


void print_position(){
  Serial.println("r: " + String(tagPosition.r) + " theta: " + String(tagPosition.theta));
}

void update_position(){
  double L = tagPosition.left_distance;
  double R = tagPosition.right_distance;
  Serial.print("L: " + String(L) + " R: " + String(R) + " ");
  double d = 0.4; //distance between the two anchors in meters 
  //TODO: fix Nan when triangle inequality doesn't hold 

  //TODO: subtract z height of the tag from calculations.

  double C_ang = acos((sq(d) + sq(L) - sq(R)) / (2 * d * L));
  //acos returns 0 to pi
  double X = cos(C_ang) * L - (d/2);
  double Y = sin(C_ang) * L;

  tagPosition.r = sqrt(sq(X) + sq(Y));
  tagPosition.theta = atan2(X, Y);
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info* info, const unsigned char *incomingData, int len) {
  // Serial.println("Length: " + String(len));
  // Serial.println("Length of anchorDistance: " + String(sizeof(anchorDistance)));
  // Serial.println("Length of packet: " + String(sizeof(packet)));
  //TODO: change the packet size of one of them (right now, they're both 8 bytes)
  // if(sizeof(packet) == len){   
  //   //TODO: CHECK IF THIS WORKS as expected
  //   packet recvd_packet;
  //   memcpy(&recvd_packet, incomingData, sizeof(recvd_packet));
  //   // Serial.println("recv turn: " + String(recvd_packet.turn) + " cur turn: " + String(cur_state.turn) + " rec for: " + String(recvd_packet.forward_speed) + " cur for: " + String(cur_state.forward_speed));
  //   if(recvd_packet.turn == cur_state.turn && recvd_packet.forward_speed == cur_state.forward_speed){
  //     ack = true;
  //   }
  // }
  // else 
  if(sizeof(anchorDistance) == len){
    anchorDistance distance_packet;
    memcpy(&distance_packet, incomingData, sizeof(distance_packet));
    if(distance_packet.id == 1)
      tagPosition.left_distance = distance_packet.distance;
    else if(distance_packet.id == 2)
      tagPosition.right_distance = distance_packet.distance;
    update_pos = true;
  }
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
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&cur_state, sizeof(cur_state));

  digitalWrite(LED_BUILTIN, LOW);
  lastSentTime = millis();
  // Serial.print("steer angle: "); Serial.println(cur_state.turn);
  // Serial.print("forward speed: "); Serial.println(cur_state.forward_speed);
  // Serial.print(millis());
  // Serial.print("\tSend Status: ");  
}

bool changed;
void setup() {

  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(BAT_MONITOR_PIN, INPUT);
  bool changed = updateButtonPressState(); //to quickly capture the button press used to wake from sleep

  Serial.begin(115200);

  //#######UWB setup
  // initialize the driver
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_SS);
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println("DW1000Ng initialized ...");
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setNetworkId(RTLS_APP_ID);

    DW1000Ng::setAntennaDelay(16436);

    DW1000Ng::applySleepConfiguration(SLEEP_CONFIG);

    DW1000Ng::setPreambleDetectionTimeout(15);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(2000);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    // char msg[128];
    // DW1000Ng::getPrintableDeviceIdentifier(msg);
    // Serial.print("Device ID: "); Serial.println(msg);
    // DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    // Serial.print("Unique ID: "); Serial.println(msg);
    // DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    // Serial.print("Network ID & Device Address: "); Serial.println(msg);
    // DW1000Ng::getPrintableDeviceMode(msg);
    // Serial.print("Device mode: "); Serial.println(msg); 
    //############


  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    ESP.restart();
    return;
  }

  // print_wakeup_reason();

  //Configure ext1 wake for deep sleep
  uint64_t bitmask = 0;
  for(int i = 0; i < sizeof(wakeup_pins) / sizeof(wakeup_pins[0]); i++){
    bitmask = bitmask | BUTTON_PIN_BITMASK(wakeup_pins[i]);
    rtc_gpio_pullup_en(wakeup_pins[i]); 
    rtc_gpio_pulldown_dis(wakeup_pins[i]); 
  }
  esp_sleep_enable_ext1_wakeup_io(bitmask, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_sleep_enable_timer_wakeup((1) * 1000000.0); //0.25 seconds = 250ms

  // Serial.println("forward: " + String(cur_state.forward_speed));
  // Serial.println("turn: " + String(cur_state.turn));

  lastEventTime = millis();
  lastSentTime = millis();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}


void batteryMonitor();
void enterDeepSleep(int wakeupTime = 0);

void loop() {
  // batteryMonitor();
  //TODO: prevent esp from going into deep sleep if tracking is enabled
  RangeInfrastructureResult res = DW1000NgRTLS::tagTwrLocalize(1500);
  if(update_pos){
    update_position();
    print_position();
    update_pos = false;
    cur_state.forward_speed = 0;
    if(tagPosition.theta > 0.1)
      cur_state.turn = 1;
    else if(tagPosition.theta < -0.1)
      cur_state.turn = -1;
    else
      cur_state.turn = 0;
    sendData();
  }

  
  
  // if(changed || !ack || millis() - lastSentTime > 250){
  //   sendData();
  //   ack = false;
  // }

  // if(ack && cur_state.forward_speed == 0 && cur_state.turn == 0){
  //   ack = false;
  //   enterDeepSleep();
  // }
  // else if(ack && cur_state.turn == 0){
  //   ack = false; //not necessary as ack gets wiped during deep sleep anyways 
  //   //go to sleep for 250ms
  //   // Serial.println("GOING TO SLEEP FOR 250MS");
  //   // Serial.println("forward: " + String(cur_state.forward_speed));
  //   // Serial.println("turn: " + String(cur_state.turn));
  //   enterDeepSleep(250);
  // }

  // else if(millis() - lastEventTime > 10000 && cur_state.turn == 0 && cur_state.forward_speed == 0){
  //   resetCommands();
  //   ack = false;
  //   //indefinite deep sleep until wakeup
  //   enterDeepSleep();
  // }

  // else if(!ack && millis() - lastEventTime > 60000){ //even if forward speed is non zero, still go to sleep
  //   resetCommands();
  //   enterDeepSleep();
  // }
  
  // last_state.forward_speed = cur_state.forward_speed; last_state.turn = cur_state.turn; last_state.set_zero_steer = cur_state.set_zero_steer;
  
  // delay(5);
  // digitalWrite(LED_BUILTIN, HIGH);
  // // Serial.println("ack: " + String(ack) + " for: " + String(cur_state.forward_speed) + " turn: " + String(cur_state.turn));

  // changed = updateButtonPressState();
}


void enterDeepSleep(int wakeupTime) {

  // Serial.println("Going to sleep with wakeup time " + String(wakeupTime));
  if(wakeupTime == 0){
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER); //disable light sleep timer
  }
  else{
    esp_sleep_enable_timer_wakeup((wakeupTime) * 1000.0); //0.25 seconds = 250ms
  }

  esp_deep_sleep_start();
}


// int speedLevel = cur_state.forward_speed / 4;  // -3 to 9 speed level, with [INSERT SOMETHING] being freespinning wheel (CODE THIS IN)
//first forward from standing start will increase it to level 4, with subsequent increments being 1
RTC_DATA_ATTR bool pressed = false;
float increment = 0.2;
bool updateButtonPressState() {
  bool stop_pin = !digitalRead(STOP_PIN), up = !digitalRead(UP_PIN),
  down = !digitalRead(DOWN_PIN), left = !digitalRead(LEFT_PIN),  right = !digitalRead(RIGHT_PIN);

  if (stop_pin && !pressed) {
    pressed = true;
    cur_state.forward_speed = 0;
     cur_state.turn  = 0;
  } else if (up && !pressed) {
    pressed = true; 
    if (cur_state.forward_speed == 0) cur_state.forward_speed = 4;
    else cur_state.forward_speed = min(26, cur_state.forward_speed + 1);
  } else if (down && !pressed) {
    pressed = true;
    cur_state.forward_speed = max(-26, cur_state.forward_speed - 1);
  }
  if (left && right && stop_pin && last_state.set_zero_steer == false) {
    cur_state.set_zero_steer = true;
  }
  else{
    cur_state.set_zero_steer = false;
    if (left) {  //turning should stop as soon as button is released
      cur_state.turn  = -1;
    } else if (right) {
      cur_state.turn  = 1;
    }
    else{
      cur_state.turn  = 0;
    }
  }

  if (!stop_pin && !up && !down) {
    pressed = false; //reset debounce
  }

  if (stop_pin && up && down && left && right){ //user can restart like this
    ESP.restart();
  }

  bool changed = !(cur_state.forward_speed == last_state.forward_speed) || !(cur_state.turn == last_state.turn) || !(cur_state.set_zero_steer == last_state.set_zero_steer);
  if (changed) {
    lastEventTime = millis();
  }

  return changed;
}

void batteryMonitor() {
  //with 1:1 voltage divider, 4.2V should be 2.1V --> 1023 * (2.1V / 3.3V) = 651
  //to go from ticks to V, 651 --> 4.2V, 0 --> 0V
  float batteryOffset = 290;  //calibrate later (detect ticks at 4.2V)
  float ticksToVoltage = 4.2 / batteryOffset;

  cur_state.remoteBatteryVoltage = 0; //analogRead(BAT_MONITOR_PIN) * ticksToVoltage;
}

void resetCommands(){
  cur_state.forward_speed = 0; cur_state.turn = 0;
  last_state.forward_speed = 0; last_state.turn = 0;
}