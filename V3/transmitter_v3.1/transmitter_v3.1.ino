#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "driver/rtc_io.h"

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

// Callback when data is received
void OnDataRecv(const esp_now_recv_info* info, const unsigned char *incomingData, int len) {
  packet recvd_packet;
  memcpy(&recvd_packet, incomingData, sizeof(recvd_packet));
  // Serial.println("recv turn: " + String(recvd_packet.turn) + " cur turn: " + String(cur_state.turn) + " rec for: " + String(recvd_packet.forward_speed) + " cur for: " + String(cur_state.forward_speed));
  if(recvd_packet.turn == cur_state.turn && recvd_packet.forward_speed == cur_state.forward_speed){
    ack = true;
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
  
  if(changed || !ack || millis() - lastSentTime > 250){
    sendData();
    ack = false;
  }

  if(ack && cur_state.forward_speed == 0 && cur_state.turn == 0){
    ack = false;
    enterDeepSleep();
  }
  else if(ack && cur_state.turn == 0){
    ack = false; //not necessary as ack gets wiped during deep sleep anyways 
    //go to sleep for 250ms
    // Serial.println("GOING TO SLEEP FOR 250MS");
    // Serial.println("forward: " + String(cur_state.forward_speed));
    // Serial.println("turn: " + String(cur_state.turn));
    enterDeepSleep(250);
  }

  else if(millis() - lastEventTime > 10000 && cur_state.turn == 0 && cur_state.forward_speed == 0){
    resetCommands();
    ack = false;
    //indefinite deep sleep until wakeup
    enterDeepSleep();
  }

  else if(!ack && millis() - lastEventTime > 60000){ //even if forward speed is non zero, still go to sleep
    resetCommands();
    enterDeepSleep();
  }
  
  last_state.forward_speed = cur_state.forward_speed; last_state.turn = cur_state.turn; last_state.set_zero_steer = cur_state.set_zero_steer;
  
  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
  // Serial.println("ack: " + String(ack) + " for: " + String(cur_state.forward_speed) + " turn: " + String(cur_state.turn));

  changed = updateButtonPressState();
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