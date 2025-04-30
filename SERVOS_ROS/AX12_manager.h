#pragma once

#include <Dynamixel.h> //https://github.com/hideakitai/Dynamixel
#include "CanStruct/can_structs.h"

#define DYNAMIXEL_SERIAL Serial2 // change as you want
#define DYNAMIXEL_SERIAL_TX_pin 17
#define DYNAMIXEL_SERIAL_RX_pin 18
const uint8_t TARGET_ID_1 = 1; // daisy-chained first dynamixel
const uint8_t TARGET_ID_2 = 2; // daisy-chained second dynamixel
const uint8_t PIN_RTS = 21;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

int dxl_goal_position_1[2];
int dxl_goal_position_2[2];

// TODO:
using namespace arduino;

void setupDynamixel()
{
    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE, SERIAL_8N1, DYNAMIXEL_SERIAL_RX_pin, DYNAMIXEL_SERIAL_TX_pin);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(TARGET_ID_1);
    dxl.addModel<DxlModel::X>(TARGET_ID_2);

    delay(2000);

    dxl.torqueEnable(TARGET_ID_1, false);
    dxl.torqueEnable(TARGET_ID_2, false);

    Serial.println("ID 1 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    Serial.println("ID 2 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    dxl.minPositionLimit(TARGET_ID_1, 1400);
    dxl.verbose(TARGET_ID_1);
    dxl.maxPositionLimit(TARGET_ID_1, 1900);
    dxl.verbose(TARGET_ID_1);

    dxl.minPositionLimit(TARGET_ID_2, 1400);
    dxl.verbose(TARGET_ID_2);
    dxl.maxPositionLimit(TARGET_ID_2, 1900);
    dxl.verbose(TARGET_ID_2);

    dxl_goal_position_1[0] = dxl.minPositionLimit(TARGET_ID_1);
    dxl_goal_position_1[1] = dxl.maxPositionLimit(TARGET_ID_1);

    dxl_goal_position_2[0] = dxl.minPositionLimit(TARGET_ID_2);
    dxl_goal_position_2[1] = dxl.maxPositionLimit(TARGET_ID_2);

    dxl.velocityLimit(TARGET_ID_1, 30000);
    dxl.velocityLimit(TARGET_ID_2, 30000);

    Serial.println("ID 1 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    Serial.println("ID 2 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    dxl.torqueEnable(TARGET_ID_1, true);
    dxl.torqueEnable(TARGET_ID_2, true);
}

void updateDynamixel(CAN::AX12Write a_ax12_msg, uint8_t ax12_id)
{
    //a_ax12_msg.mode == // by default, they are all in position mode

    dxl.goalPosition(ax12_id, a_ax12_msg.position);
    dxl.torqueEnable(ax12_id, a_ax12_msg.torque_enable);
    //dxl.velocityLimit(ax12_id, a_ax12_msg.max_speed);
    //dxl.accelerationLimit(ax12_id, a_ax12_msg.max_accel);
    //dxl.currentLimit(ax12_id, a_ax12_msg.currentLimit);
    //dxl.temperatureLimit(ax12_id, a_ax12_msg.temperatureLimit);
}