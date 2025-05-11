#pragma once

#include <Dynamixel.h> //https://github.com/hideakitai/Dynamixel
#include "CanStruct/can_structs.h"

#define DYNAMIXEL_SERIAL Serial2 // change as you want
#define DYNAMIXEL_SERIAL_TX_pin 17
#define DYNAMIXEL_SERIAL_RX_pin 18
#define NB_AX12 4
const uint8_t PIN_RTS = 21;
const uint32_t DYNAMIXEL_BAUDRATE = 1000000;

Dynamixel dxl(PIN_RTS);

struct AX12
{
    int8_t id;
    CAN::AX12Read infos;
    CAN::AX12Write commands;

    AX12(int8_t a_id) : id(a_id)
    {
        commands.torque_enable = 0;
        commands.currentLimit = 0;
        commands.max_accel = 0;
        commands.position = 0;
        commands.temperatureLimit = 65;
        commands.mode = 0;
        commands.max_speed = 0;

        infos.current_position = 0;
        infos.presentCurrent = 0;
        infos.presentTemperature = 0;
        infos.hardwareErrorStatus = 0;
        infos.moving = 0;
        infos.mode = 1;
    }
};

AX12 myAX12s[NB_AX12 + 2] = {AX12(1), AX12(2), AX12(3), AX12(4), AX12(5), AX12(6)};

using namespace arduino;

void setupDynamixel()
{
    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE, SERIAL_8N1, DYNAMIXEL_SERIAL_RX_pin, DYNAMIXEL_SERIAL_TX_pin);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);

    for (int i = 0; i < NB_AX12; i++)
    {
        dxl.addModel<DxlModel::X>(myAX12s[i].id);
    }
}

void updateDynamixel(CAN::AX12Write a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode

    dxl.goalPosition(ax12_id, a_ax12_msg.position);
    dxl.torqueEnable(ax12_id, a_ax12_msg.torque_enable);
    if (a_ax12_msg.currentLimit != -1)
    {
        dxl.currentLimit(ax12_id, a_ax12_msg.currentLimit);
    }
    if (a_ax12_msg.max_speed != -1)
    {
        dxl.velocityLimit(ax12_id, a_ax12_msg.max_speed);
    }
    if (a_ax12_msg.temperatureLimit != -1)
    {
        dxl.temperatureLimit(ax12_id, a_ax12_msg.temperatureLimit);
    }
    if (a_ax12_msg.max_accel != -1)
    {
        dxl.accelerationLimit(ax12_id, a_ax12_msg.max_accel);
    }
    // dxl.velocityLimit(ax12_id, a_ax12_msg.max_speed);
    // dxl.accelerationLimit(ax12_id, a_ax12_msg.max_accel);
    // dxl.currentLimit(ax12_id, a_ax12_msg.currentLimit);
    // dxl.temperatureLimit(ax12_id, a_ax12_msg.temperatureLimit);
}

void updateDynamixelInfo(CAN::AX12Read &a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode

    a_ax12_msg.current_position = dxl.presentPosition(ax12_id);
    a_ax12_msg.presentCurrent = dxl.presentCurrent(ax12_id);
    a_ax12_msg.presentTemperature = dxl.presentTemperature(ax12_id);
    a_ax12_msg.hardwareErrorStatus = dxl.hardwareErrorStatus(ax12_id);
    a_ax12_msg.moving = dxl.movingStatus(ax12_id);
    a_ax12_msg.mode = 1; // dxl.mode(ax12_id);
}

void updateDynamixels()
{
    for (int i = 0; i < NB_AX12; i++)
    {
        updateDynamixel(myAX12s[i].commands, myAX12s[i].id);
    }
}

void updateDynamixelsInfo()
{
    for (int i = 0; i < NB_AX12; i++)
    {
        updateDynamixelInfo(myAX12s[i].infos, myAX12s[i].id);
    }
}