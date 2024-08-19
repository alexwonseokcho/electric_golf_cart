#include <Wire.h>

struct anchor_packet {
  double distance;
  uint8_t anchor_num;
};

anchor_packet anchorDataPacket = {0.0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(6, 7, 50000);
  // Wire.setClock(100000);
  Serial.println("starting");
}

unsigned long lastTime = millis();

void loop() {

  // Serial.println(millis());
  // Wire.beginTransmission(1);                // start transmit to slave (8)
  // Wire.write(0x23);                                 // sends 6 bytes value to slave
  // delay(2000);
  // Wire.write(0x23);                                 // sends 6 bytes value to 
  // delay(2000);
  // Wire.endTransmission();                   // stop transmitting


  Wire.requestFrom(1, sizeof(anchorDataPacket));                   // request 13 bytes from slave (8)
  // int i = 0;
  Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket)); //is this blocking?? if it is that's no bueno 
  
  Serial.println(anchorDataPacket.distance);
  Serial.println(anchorDataPacket.anchor_num);

   Wire.requestFrom(2, sizeof(anchorDataPacket));                   // request 13 bytes from slave (8)
  // i = 0;
  Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket)); //is this blocking?? if it is that's no bueno 

  Serial.println(anchorDataPacket.distance);
  Serial.println(anchorDataPacket.anchor_num);

  Wire.requestFrom(1, sizeof(anchorDataPacket));                   // request 13 bytes from slave (8)
  // int i = 0;
  Wire.readBytes((byte*)&anchorDataPacket, sizeof(anchorDataPacket)); //is this 
  
  Serial.println(anchorDataPacket.distance);
  Serial.println(anchorDataPacket.anchor_num);

  // Serial.println(); 
  Serial.println("update rate: " + String(1000.0 / (millis() - lastTime)));
  lastTime = millis();
  // delay(500);
}
