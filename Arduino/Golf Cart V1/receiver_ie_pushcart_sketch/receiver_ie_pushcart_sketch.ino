// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      500         // [-] Maximum speed for testing
#define SPEED_STEP          5          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX
RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;


void setup() 
{
  Serial.begin(SERIAL_BAUD);
   Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive() //add code to check for failure modes & display that on controller
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
      incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
      bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
      
    }
    else {
      return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("cmd1: ");   Serial.print(Feedback.cmd1);
            Serial.print("cmd2: ");  Serial.print(Feedback.cmd2);
            Serial.print("SpeedR meas: ");  Serial.print(Feedback.speedR_meas);
            Serial.print("SpeedL meas: ");  Serial.print(Feedback.speedL_meas);
            Serial.print("Bat V: ");  Serial.print(Feedback.batVoltage);
            Serial.print("board Temp: ");  Serial.print(Feedback.boardTemp);
            Serial.print("cmd LED: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

int forward = 0;
int lastForward = 0;
int steer = 0;
int remoteBatteryVoltage = 0;
unsigned long int lastReceivedTime = 0;
void receive_nRF()
{
  int rec_data[4];
//  int rec_data;
//  Serial.println("Radio");
  if (radio.available()){
    radio.read(&rec_data, sizeof(rec_data));
//    Serial.println("Received Data: " + rec_data[0] + rec_data[1] + rec_data[2] + rec_data[3]);


    forward = rec_data[0];
    steer = rec_data[1];
    remoteBatteryVoltage = rec_data[2];
    
    //something = rec_data[3];
    lastReceivedTime = millis();
  }
}

bool is_nRF_connected(){ //not tested
  if(lastReceivedTime != 0 && millis() - lastReceivedTime > 1000)
    return false;
  return true;
}

// ########################## LOOP ##########################
void loop(void)
{ 
  // Check for new received data
  Receive();

  receive_nRF();
  
  //make it stop when radio disconnects IMPORTANT VERY VERY IMPORTANT
  //solder capacitors to the nRF modules
  if(is_nRF_connected() == false){
    forward = 0;
    steer = 0;
    lastForward = 0;
    Serial.println("NOT CONNECTED");
  }

  

//  Serial.println(forward);
//  Serial.println(steer);
//  Serial.println(remoteBatteryVoltage);

  //Low pass filter for accelerating forward
  if(forward > lastForward) 
    forward = min(lastForward + 5, forward);

//  Serial.print("Real motor forward value: ");
//  Serial.println(forward);
  Send(steer, forward); // steer, forward 

  lastForward = forward;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (millis()%2000)<1000);
}

// ########################## END ##########################
