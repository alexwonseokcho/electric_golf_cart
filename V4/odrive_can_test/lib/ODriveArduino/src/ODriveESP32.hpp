#pragma once

#include <ESP32-TWAI-CAN.hpp>
#include "ODriveCAN.h"

// This is a convenience struct because the MCP2515 library doesn't have a
// native message type.
//struct CanMsg {
//    uint32_t id;
//    uint8_t len;
//    uint8_t buffer[8];
//};

using CanMsg = CanFrame;

// Must be defined by the application if you want to use defaultCanReceiveCallback().
void onCanMessage(const CanFrame& msg);

static bool sendMsg(TwaiCAN& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    CanFrame msg = { 0 };
    msg.identifier = id & 0x1fffffff;
    msg.extd = id & 0x80000000;
    msg.data_length_code = length;
    
    memcpy(msg.data, &data, sizeof(uint8_t) * length);
    
    return can_intf.writeFrame(msg);
}

static void onReceive(const CanFrame& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.identifier, msg.data_length_code, msg.data);
}

static void pumpEvents(TwaiCAN& intf) {
    CanFrame temp;
    if(intf.readFrame(temp, 5)){
        onCanMessage(temp);
        // Serial.println("Received CAN message");
    }
    // Serial.println("No CAN message received");
    // nothing to do
    // TODO: maybe remove
    // delay(10); // not sure why this resulted in less dropped messages, could have been a twisted coincidence
}

CREATE_CAN_INTF_WRAPPER(TwaiCAN)
