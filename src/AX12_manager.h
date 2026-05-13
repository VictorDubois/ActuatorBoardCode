#pragma once
#include "Arduino.h"
// #include <Dynamixel.h> //https://github.com/hideakitai/Dynamixel => does not seem to work
#include <Dynamixel2Arduino.h>
#include "ESP32SerialPortHandler.cpp"

#include "CanStruct/can_structs.h"

#define DYNAMIXEL_SERIAL Serial2 // change as you want
#define DYNAMIXEL_SERIAL_TX_pin 17
#define DYNAMIXEL_SERIAL_RX_pin 18
#define NB_AX12 4
#define AX12_A_MODEL_ID 12
const uint8_t PIN_RTS = 21;
const uint32_t DYNAMIXEL_BAUDRATE = 1000000;
// const uint32_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel2Arduino dxl;
ESP32SerialPortHandler esp_dxl_port(DYNAMIXEL_SERIAL, DYNAMIXEL_SERIAL_RX_pin, DYNAMIXEL_SERIAL_TX_pin, PIN_RTS);

uint8_t sequenced_updates_servo = 0;      // Do not update everything all at once
uint8_t sequenced_updates_subcommand = 0; // Do not update everything all at once

enum AX12_subcommand
{
    position = 0,
    torque_enable = 1,
    max_current = 2,
    max_speed = 3,
    end_of_commands = 4,
    // Not used:
    max_accel = 5,
    max_temperature = 6
};

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

AX12 myAX12s[NB_AX12 + 2] = {AX12(3), AX12(4), AX12(7), AX12(6), AX12(21), AX12(22)};

void setupDynamixel()
{
    pinMode(DYNAMIXEL_SERIAL_TX_pin, OUTPUT);
    pinMode(DYNAMIXEL_SERIAL_RX_pin, INPUT);
    pinMode(PIN_RTS, OUTPUT);
    digitalWrite(PIN_RTS, HIGH);

    dxl.setPort(esp_dxl_port);
    usleep(20000);

    dxl.setPortProtocolVersion(1);
    usleep(20000);

    dxl.begin(DYNAMIXEL_BAUDRATE);
    usleep(20000);

    for (int i = 0; i < NB_AX12; i++)
    {
        auto id = myAX12s[i].id;

        // dxl.setModelNumber(id, AX12_A_MODEL_ID);
        usleep(20000);

        Serial.print("getModelNumberFromTable: ");
        Serial.println(dxl.getModelNumberFromTable(id));
        if (dxl.ping(id))
        {
            usleep(20000);

            Serial.print("Found dynamixel ");
            Serial.println(id);
        }
        else
        {
            usleep(20000);

            Serial.print("Missing dynamixel ");
            Serial.println(id);
        }
    }
}

void updateDynamixel(CAN::AX12Write a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode
    uint8_t i_subcommand = sequenced_updates_subcommand;
    sequenced_updates_subcommand++;
    if (sequenced_updates_subcommand >= AX12_subcommand::end_of_commands)
    {
        sequenced_updates_subcommand = 0;
    }
    if (a_ax12_msg.currentLimit > 100)
    {
        a_ax12_msg.currentLimit = 100;
    }

    if (a_ax12_msg.max_speed > 100)
    {
        a_ax12_msg.max_speed = 100;
    }

    if (ax12_id == 8)
    {
        // Ce servo reçoit parfois des messges buggés
        a_ax12_msg.max_speed = 10;
        if (a_ax12_msg.position > 400)
        {
            a_ax12_msg.position = 400;
        }
        if (a_ax12_msg.position < 100)
        {
            a_ax12_msg.position = 100;
        }
    }

    dxl.setGoalPosition(ax12_id, a_ax12_msg.position);
    if (dxl.getLastLibErrCode() != 0)
    {
        Serial.print("Error setting position for AX12 ID ");
        Serial.print(ax12_id);
        Serial.print(": ");
        Serial.println(dxl.getLastLibErrCode());
    }
    usleep(1000);

    if (a_ax12_msg.torque_enable)
    {
        dxl.torqueOn(ax12_id);
        if (dxl.getLastLibErrCode() != 0)
        {
            Serial.print("Error setting torqueOn for AX12 ID ");
            Serial.print(ax12_id);
            Serial.print(": ");
            Serial.println(dxl.getLastLibErrCode());
        }
    }
    else
    {
        dxl.torqueOff(ax12_id);
        if (dxl.getLastLibErrCode() != 0)
        {
            Serial.print("Error setting torqueOff for AX12 ID ");
            Serial.print(ax12_id);
            Serial.print(": ");
            Serial.println(dxl.getLastLibErrCode());
        }
    }
    usleep(1000);

    if (a_ax12_msg.max_accel != -1)
    {
        // dxl.setAccelerationLimit(ax12_id, a_ax12_msg.max_accel);
    }

    if (a_ax12_msg.max_speed != -1)
    {
        dxl.setGoalVelocity(ax12_id, a_ax12_msg.max_speed, UNIT_PERCENT);
        if (dxl.getLastLibErrCode() != 0)
        {
            Serial.print("Error setting velocity for AX12 ID ");
            Serial.print(ax12_id);
            Serial.print(": ");
            Serial.println(dxl.getLastLibErrCode());
        }
    }
    usleep(1000);

    // if (a_ax12_msg.currentLimit != -1)
    // {
    //     dxl.setGoalCurrent(ax12_id, a_ax12_msg.currentLimit, UNIT_PERCENT);
    //     if (dxl.getLastLibErrCode() != 0)
    //     {
    //         Serial.print("Error setting current for AX12 ID ");
    //         Serial.print(ax12_id);
    //         Serial.print(": ");
    //         Serial.println(dxl.getLastLibErrCode());
    //     }
    // }

    if (a_ax12_msg.currentLimit != -1)
    {
        if (a_ax12_msg.currentLimit > 100)
        {
            a_ax12_msg.currentLimit = 100;
        }
        // AX-12A Torque Limit: address 34, 2 bytes, range 0-1023
        uint16_t torque_limit = (uint16_t)(a_ax12_msg.currentLimit * 1023 / 100);
        dxl.write(ax12_id, 34, (uint8_t*)&torque_limit, 2);
        if (dxl.getLastLibErrCode() != 0)
        {
            Serial.print("Error setting torque limit for AX12 ID ");
            Serial.print(ax12_id);
            Serial.print(": ");
            Serial.println(dxl.getLastLibErrCode());
        }
    }
    
    if (a_ax12_msg.temperatureLimit != -1)
    {
        // dxl.setTemperatureLimit(ax12_id, a_ax12_msg.temperatureLimit);
    }
}

void updateDynamixelInfo(CAN::AX12Read &a_ax12_msg, uint8_t ax12_id)
{
    // a_ax12_msg.mode == // by default, they are all in position mode
    usleep(10000);
    float l_position = dxl.getPresentPosition(ax12_id, UNIT_RAW);
    a_ax12_msg.current_position = static_cast<uint16_t>(l_position);
    usleep(10000);
    l_position = dxl.getPresentPosition(ax12_id, UNIT_RAW); // Second read because the first one is often wrong for some reason
    a_ax12_msg.current_position = static_cast<uint16_t>(l_position);
    if (dxl.getLastLibErrCode() != 0)
    {
        Serial.print("Error getting position for AX12 ID ");
        Serial.print(ax12_id);
        Serial.print(": ");
        Serial.println(dxl.getLastLibErrCode());
    }

    usleep(10000);

    a_ax12_msg.presentCurrent = -1;
    // dxl.getPresentCurrent(ax12_id); // Not supported by AX12
    a_ax12_msg.presentTemperature = -1; // dxl.getPresentTemperature(ax12_id);
    a_ax12_msg.hardwareErrorStatus = 0; // dxl.hardwareErrorStatus(ax12_id);
    float l_velocity = dxl.getPresentVelocity(ax12_id, UNIT_RAW);
    a_ax12_msg.moving = static_cast<uint8_t>(l_velocity / 10);
    usleep(10000);
    l_velocity = dxl.getPresentVelocity(ax12_id, UNIT_RAW); // Second read because the first one is often wrong for some reason
    a_ax12_msg.moving = static_cast<uint8_t>(l_velocity / 10);
    if (dxl.getLastLibErrCode() != 0)
    {
        Serial.print("Error getting velocity for AX12 ID ");
        Serial.print(ax12_id);
        Serial.print(": ");
        Serial.println(dxl.getLastLibErrCode());
    }

    a_ax12_msg.mode = 1; // dxl.mode(ax12_id);
    usleep(10000);
}

void updateDynamixels()
{
    uint8_t i_servo = sequenced_updates_servo;
    updateDynamixel(myAX12s[i_servo].commands, myAX12s[i_servo].id);
    sequenced_updates_servo++;
    if (sequenced_updates_servo >= NB_AX12)
    {
        sequenced_updates_servo = 0;
    }
}

void updateDynamixelsInfo()
{
    for (int i = 0; i < NB_AX12; i++)
    {
        updateDynamixelInfo(myAX12s[i].infos, myAX12s[i].id);
    }
}