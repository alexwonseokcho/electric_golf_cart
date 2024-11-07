#include <SoftwareSerial.h>

EspSoftwareSerial::UART anc1Port;
EspSoftwareSerial::UART anc2Port;
EspSoftwareSerial::UART anc3Port;

struct anchorReading{
  float anc1;
  float anc2;
  float anc3;
};

anchorReading anchorReading = {0, 0, 0};

void setup() {
  anc1Port.begin(9600, SWSERIAL_8N1, D3, -1);
  if (!anc1Port) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Anchor 1 couldn't connect"); 
    ESP.restart();
  } 

  anc2Port.begin(9600, SWSERIAL_8N1, D2, -1);
  if (!anc2Port) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Anchor 2 couldn't connect"); 
    ESP.restart();
  } 

  anc3Port.begin(9600, SWSERIAL_8N1, D1, -1);
  if (!anc3Port) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Anchor 2 couldn't connect"); 
    ESP.restart();
  } 

  Serial.begin(9600);
  Serial.println("Hello");
}

int lastRecvTimes[3] = {millis(), millis(), millis()};

void loop() {

    static char anc1_buffer[32];
    static size_t anc1_pos;
    if (anc1Port.available()) {
      char c = anc1Port.read();
      if (c == '\n') {  // on end of line, parse the number
          anc1_buffer[anc1_pos] = '\0';
          anchorReading.anc1 = atof(anc1_buffer);
          anc1_pos = 0;
      } else if (anc1_pos < sizeof(anc1_buffer) - 1) {  // otherwise, buffer it
          anc1_buffer[anc1_pos++] = c;
      }
      lastRecvTimes[0] = millis();
    }
    
    static char anc2_buffer[32];
    static size_t anc2_pos;
    if (anc2Port.available()) {
        char c = anc2Port.read();
        if (c == '\n') {  // on end of line, parse the number
            anc2_buffer[anc2_pos] = '\0';
            anchorReading.anc2 = atof(anc2_buffer);
            anc2_pos = 0;
        } else if (anc2_pos < sizeof(anc2_buffer) - 1) {  // otherwise, buffer it
            anc2_buffer[anc2_pos++] = c;
        }
        lastRecvTimes[1] = millis();
    }

    static char anc3_buffer[32];
    static size_t anc3_pos;
    if (anc3Port.available()) {
        char c = anc3Port.read();
        if (c == '\n') {  // on end of line, parse the number
            anc3_buffer[anc3_pos] = '\0';
            anchorReading.anc3 = atof(anc3_buffer);
            anc3_pos = 0;
        } else if (anc3_pos < sizeof(anc3_buffer) - 1) {  // otherwise, buffer it
            anc3_buffer[anc3_pos++] = c;
        }
        lastRecvTimes[2] = millis();
    }

    areAnchorsAlive();

    Serial.print(anchorReading.anc1);
    Serial.print("\t");
    Serial.print(anchorReading.anc2);
    Serial.print("\t");
    Serial.println(anchorReading.anc3);
}

void areAnchorsAlive(){
  for(int i = 0; i < 3; i++){
    if(millis() - lastRecvTimes[i] > 1000){
      Serial.println("ANCHOR #" + (String) i + " DISCONNECTED"); //TODO: make this actually error out & stop follow me
    }
  }
}
