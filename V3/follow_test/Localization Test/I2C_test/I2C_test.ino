// #include <Wire.h>

// double anchor1_dist = 0;
// double anchor2_dist = 0;
// double anchor3_dist = 0;

// void setup() {
//   Serial.begin(9600);
//   Wire.begin(5, 6, 100000); //SDA, SCL, D2, D3
//   Serial.print("dslkfjsdlkfjsl");
  
// }

// void loop() {
//   // for(int i = 1; i < 4; i++){
//     Wire.requestFrom(1, 1); //address, size
//     double distance = 0;
//     while (Wire.available())
//     {
//       distance = Wire.read();
//       Serial.print(distance);
//       // wholeData += inChr;
//     }

//   // }
// }

// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
 
#include <SoftwareSerial.h>

 
void setup()
{
  Software
}
 
 
void loop()
{
  Wire.requestFrom(0x29, 1);
  if(Wire.available()){
    Serial.println(Wire.read());
  }
}