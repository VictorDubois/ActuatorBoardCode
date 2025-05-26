#pragma once
#include "Arduino.h"
// #include <Dynamixel.h> //https://github.com/hideakitai/Dynamixel => does not seem to work
#include <Dynamixel2Arduino.h>
#include "ESP32SerialPortHandler.cpp"

#include "CanStruct/can_structs.h"

#define DYNAMIXEL_SERIAL Serial2 // change as you want
#define DYNAMIXEL_SERIAL_TX_pin 17
#define DYNAMIXEL_SERIAL_RX_pin 18
#define NB_AX12 3
#define AX12_A_MODEL_ID 12
const uint8_t PIN_RTS = 21;
const uint32_t DYNAMIXEL_BAUDRATE = 1000000;
// const uint32_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel2Arduino dxl;
ESP32SerialPortHandler esp_dxl_port(DYNAMIXEL_SERIAL, DYNAMIXEL_SERIAL_RX_pin, DYNAMIXEL_SERIAL_TX_pin, PIN_RTS);

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

AX12 myAX12s[NB_AX12 + 3] = {AX12(1), AX12(2), AX12(3), AX12(4), AX12(5), AX12(6)};

void setupDynamixel()
{
    pinMode(DYNAMIXEL_SERIAL_TX_pin, OUTPUT);
    pinMode(DYNAMIXEL_SERIAL_RX_pin, INPUT);
    pinMode(PIN_RTS, OUTPUT);
    digitalWrite(PIN_RTS, HIGH);

    dxl.setPort(esp_dxl_port);
    dxl.setPortProtocolVersion(1);
    dxl.begin(DYNAMIXEL_BAUDRATE);

    for (int i = 0; i < NB_AX12; i++)
    {
        dxl.setModelNumber(i, AX12_A_MODEL_ID);
        auto id = myAX12s[i].id;
        if (dxl.ping())
        {
            Serial.print("Found dynamixel ");
            Serial.println(id);
        }
        else
        {
            Serial.print("Missing dynamixel ");
            Serial.println(id);
        }
    }
}

void updateDynamixel(CAN::AX12Write a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode

    if (a_ax12_msg.currentLimit > 100)
    {
        a_ax12_msg.currentLimit = 100;
    }

    if (a_ax12_msg.max_speed > 100)
    {
        a_ax12_msg.max_speed = 100;
    }

    dxl.setGoalPosition(ax12_id, a_ax12_msg.position);
    if (a_ax12_msg.torque_enable)
    {
        dxl.torqueOn(ax12_id);
    }
    else
    {
        dxl.torqueOff(ax12_id);
    }
    if (a_ax12_msg.currentLimit != -1)
    {
        dxl.setGoalCurrent(ax12_id, a_ax12_msg.currentLimit, UNIT_PERCENT);
    }
    if (a_ax12_msg.max_speed != -1)
    {
        dxl.setGoalVelocity(ax12_id, a_ax12_msg.max_speed, UNIT_PERCENT);
    }
    if (a_ax12_msg.temperatureLimit != -1)
    {
        // dxl.setTemperatureLimit(ax12_id, a_ax12_msg.temperatureLimit);
    }
    if (a_ax12_msg.max_accel != -1)
    {
        // dxl.setAccelerationLimit(ax12_id, a_ax12_msg.max_accel);
    }
}

void updateDynamixelInfo(CAN::AX12Read &a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode

    a_ax12_msg.current_position = dxl.getPresentPosition(ax12_id);
    a_ax12_msg.presentCurrent = dxl.getPresentCurrent(ax12_id);
    a_ax12_msg.presentTemperature = -1; // dxl.getPresentTemperature(ax12_id);
    a_ax12_msg.hardwareErrorStatus = 0; // dxl.hardwareErrorStatus(ax12_id);
    a_ax12_msg.moving = dxl.getPresentVelocity(ax12_id);
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