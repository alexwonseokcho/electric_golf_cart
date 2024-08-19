#include <Wire.h>

struct anchor_packet {
  double distance;
  uint8_t anchor_num;
};

anchor_packet anchorDataPacket = {0, 1};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(1, 0, 1, 50000); //addr, SDA, SCL, freq
  // Wire.setClock(100000); //100000 Hz
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
  Serial.println("hello");
}

void onRequest(){
  // uint8_t text[] = {'a', 'b', 'c', 'd'};
  anchorDataPacket.distance = millis();
  Wire.write((byte *)&anchorDataPacket, sizeof(anchorDataPacket));
  // Serial.println("sent 2");
}

void onReceive(int numBytes){
  Serial.print("Received");
  Serial.print(numBytes);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("im alive 1");
  delay(1000);
}
