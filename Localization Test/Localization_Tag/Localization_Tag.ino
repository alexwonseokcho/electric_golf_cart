/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSTag_TWR.ino
 * 
 * This is an example tag in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

// C3 SUPERMINI pins
const uint8_t PIN_RST = 20; // reset pin
const uint8_t PIN_IRQ = 21; // irq pin
const uint8_t PIN_SS = 7; // spi select pin

const uint8_t PIN_MOSI = 10;
const uint8_t PIN_MISO = 9;
const uint8_t PIN_CLK = 8;
// const uint8_t PIN_RST = 5; // reset pin
// const uint8_t PIN_IRQ = 6; // irq pin
// const uint8_t PIN_SS = 1; // spi select pin

// const uint8_t PIN_MOSI = 4;
// const uint8_t PIN_MISO = 3;
// const uint8_t PIN_CLK = 2;


// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:00";

volatile uint32_t blink_rate = 200;

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

void setup() {
    // DEBUG monitoring
    delay(50);
    Serial.begin(115200);
    Serial.println("dlskjflsdf'");
    Serial.println(F("### DW1000Ng-arduino-ranging-tag ###"));
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
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);    
}

void loop() {
    // DW1000Ng::deepSleep();
    // delay(blink_rate);
    // DW1000Ng::spiWakeup();

    DW1000Ng::setEUI(EUI);

    RangeInfrastructureResult res = DW1000NgRTLS::tagTwrLocalize(1500);
    delay(20);
    // if(res.success)
    //     blink_rate = res.new_blink_rate;
}
