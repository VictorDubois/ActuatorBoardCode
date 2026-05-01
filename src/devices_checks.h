#pragma once

#include "constants.h"
#include "io_expender_manager.h"
#include "Arduino.h"
#include "AX12_manager.h"
#include "servos_manager.h"
#include "stepper_manager.h"
#include "oled_manager.h"

void testTransistors(bool doTest, IOPotentiallyExpended io)
{
    if (!doTest)
    {
        return;
    }

    while (true)
    {
        Serial.println("Test transistors");
        io.myDigitalWrite(TRANSISTOR_1_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_2_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_3_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_4_PIN, HIGH);
        delay(5000);
        io.myDigitalWrite(TRANSISTOR_1_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_2_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_3_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_4_PIN, LOW);
        delay(1000);
    }
}

void testDynamixels(bool doTest)
{
    if (!doTest)
    {
        return;
    }
    int AX12_ID = 7;
    usleep(3e6);
    setupDynamixel();

    uint8_t positionServo = 0;

    // Test dynamixel
    while (true)
    {
        positionServo += 10;
        dxl.ledOn(AX12_ID);
        Serial.println("AX12 LED on");
        Serial.flush();
        delay(1000);

        dxl.ledOff(AX12_ID);
        Serial.println("AX12 LED off");
        Serial.flush();
        delay(1000);

        Serial.print("getModelNumberFromTable: ");
        Serial.println(dxl.getModelNumberFromTable(AX12_ID));

        for (int i = 0; i < NB_AX12; i++)
        {
            updateDynamixelInfo(myAX12s[i].infos, myAX12s[i].id);

            Serial.print("dxl.getLastLibErrCode() = ");
            Serial.println(dxl.getLastLibErrCode());

            Serial.print("AX12 ID: ");
            Serial.print(myAX12s[i].id);
            Serial.print(", position: ");
            Serial.print(myAX12s[i].infos.current_position);
            Serial.print(", current: ");
            Serial.print(myAX12s[i].infos.presentCurrent);
            Serial.print(", temperature: ");
            Serial.print(myAX12s[i].infos.presentTemperature);
            Serial.print(", hardwareErrorStatus: ");
            Serial.print(myAX12s[i].infos.hardwareErrorStatus);
            Serial.print(", moving: ");
            Serial.print(myAX12s[i].infos.moving);
            Serial.print(", mode: ");
            Serial.print(myAX12s[i].infos.mode);
            Serial.println();

            myAX12s[i].commands.position = positionServo + 100 + 20 * i;
            myAX12s[i].commands.currentLimit = 100;
            myAX12s[i].commands.max_accel = 100;
            myAX12s[i].commands.max_speed = 100;
            myAX12s[i].commands.torque_enable = 1;
            updateDynamixel(myAX12s[i].commands, myAX12s[i].id);

            usleep(1e6);
        }
        Serial.println();
    }
}

void testDynamixels2(bool doTest)
{
    if (!doTest)
    {
        return;
    }
    int AX12_ID = 8;
    // dxl.torqueOn(1);
    while (true)
    {
        CAN::AX12Write l_ax12_msg;
        l_ax12_msg.currentLimit = 100;
        l_ax12_msg.max_accel = 100;
        l_ax12_msg.max_speed = 100;
        l_ax12_msg.temperatureLimit = 65;
        l_ax12_msg.position = 100;
        l_ax12_msg.torque_enable = 1;

        for (int i = 0; i < 10; i++)
        {
            // myAX12s[0].commands.position(300);
            l_ax12_msg.torque_enable = 1;
            l_ax12_msg.position = 300 + 50 * i;
            l_ax12_msg.currentLimit = 250;
            l_ax12_msg.max_accel = 250;
            l_ax12_msg.max_speed = 250;
            if (i % 2)
            {
                dxl.ledOn(AX12_ID);
            }
            else
            {
                dxl.ledOff(AX12_ID);
            }
            // dxl.setGoalPosition(1, l_ax12_msg.position);

            updateDynamixel(l_ax12_msg, AX12_ID);
            Serial.println(l_ax12_msg.position);
            Serial.flush();

            delay(1000);
        }
    }
}

void testDynamixels3(bool doTest)
{
    if (!doTest)
    {
        return;
    }

    setupDynamixel();
    dxl.torqueOn(8);
    usleep(20000);

    while (true)
    {
        dxl.ledOn(8);
        usleep(20000);

        Serial.println("AX12 LED on");
        Serial.flush();
        dxl.setGoalPosition(1, 400);
        usleep(20000);

        delay(1000);

        dxl.ledOff(8);
        usleep(20000);

        dxl.setGoalPosition(1, 800);
        usleep(20000);

        Serial.println("AX12 LED off");
        Serial.flush();
        delay(1000);
    }
}

void testServos(bool doTest, PersistentServo *myPersistentServos[])
{
    if (!doTest)
    {
        return;
    }

    digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);

    // myPersistentServos[7]->speed = 128;

    while (true)
    {
        Serial.println("128");
        myPersistentServos[0]->angle = 0;   // 0 = endroit tout à droite
        myPersistentServos[1]->angle = 0;   // 0 = endroit milieu droite
        myPersistentServos[2]->angle = 0;   // 0 = endroit milieu gauche
        myPersistentServos[3]->angle = 0;   // 0 = endroit tout à gauche
        myPersistentServos[4]->angle = 175; // 175 = doigt en bas
        myPersistentServos[5]->angle = 150;
        myPersistentServos[6]->angle = 150;
        // myPersistentServos[7]->angle = 128;

        for (int l_i = 0; l_i < 100; l_i++)
        {
            updateServos(myPersistentServos);
            delay(30);
        }

        Serial.println("70");
        myPersistentServos[0]->angle = 180; // 180 = autre sens tout à droite
        myPersistentServos[1]->angle = 180; // 180 = autre sens milieu droite
        myPersistentServos[2]->angle = 180; // 180 = autre sens milieu gauche
        myPersistentServos[3]->angle = 180; // 180 = autre sens tout à gauche
        myPersistentServos[4]->angle = 125; // 110 doigt à l'horizontale
        myPersistentServos[5]->angle = 125;
        myPersistentServos[6]->angle = 125;
        // myPersistentServos[7]->angle = 128;

        // full droite 125 attrape 208 relache (testeur)
        // milieu droite 107 attrape 198 relache (testeur)
        // milieu gauche 125 attrape 210 relache (testeur)
        // full gauche 115 attrape 205 relache (testeur)
        // doigt bas 230 haut 168 (testeur)

        for (int l_i = 0; l_i < 100; l_i++)
        {
            updateServos(myPersistentServos);
            delay(30);
        }
    }
}

void testPumps(bool doTest, IOPotentiallyExpended io)
{
    if (!doTest)
    {
        return;
    }

    while (true)
    {
        io.myDigitalWrite(TRANSISTOR_1_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_2_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_3_PIN, HIGH);
        io.myDigitalWrite(TRANSISTOR_4_PIN, HIGH);
        Serial.println("HIGH");

        delay(1000);

        io.myDigitalWrite(TRANSISTOR_1_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_2_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_3_PIN, LOW);
        io.myDigitalWrite(TRANSISTOR_4_PIN, LOW);
        Serial.println("LOW");

        delay(2000);
    }
}

void testStepper(bool doTest, Stepper stepperStructHoming, IOPotentiallyExpended io)
{
    if (!doTest)
    {
        return;
    }

    stepperStructHoming.mode = stepper_mode::POSITION;

    while (true)
    {
        stepperStructHoming.position = 2; // mm 235 mm

        if ((millis() / 5000) % 2)
        {
            stepperStructHoming.position = 10; // mm 252.5 mm
            Serial.println(100);
        }
        else
        {
            Serial.println(0);
        }
        loopStepper(stepperStructHoming, io);
    }
}

void testOled(bool doTest, int current_score = 0, int remaining_time_s = 90, bool teamIsBlue = true)
{
    if (!doTest)
    {
        return;
    }

    while (true)
    {
        uint16_t bat_power_voltage_int = analogRead(BAT_POWER_PIN);
        uint16_t bat_elec_voltage_int = analogRead(BAT_ELEC_PIN);
        float bat_power_voltage = bat_power_voltage_int * 6.935283019 * 3300.0f / 4096.0f; // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )
        float bat_elec_voltage = bat_elec_voltage_int * 7.3514 * 3300.0f / 4096.0f;        // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )

        Serial.print("Power: ");
        Serial.print(bat_power_voltage_int);
        Serial.print(", ");
        Serial.println(bat_power_voltage);
        Serial.print(", ");
        Serial.println(batt_to_str(bat_power_voltage));

        Serial.print("Elec: ");
        Serial.print(bat_elec_voltage_int);
        Serial.print(", ");
        Serial.println(bat_elec_voltage);
        Serial.print(", ");
        Serial.println(batt_to_str(bat_elec_voltage));

        // drawLCD(current_score);
        // current_score = (millis()/100)%256;
        loopOled(current_score, remaining_time_s, teamIsBlue, bat_elec_voltage, bat_power_voltage);
    }
}

void testIOs(bool doTest, IOPotentiallyExpended io)
{
    if (!doTest)
    {
        return;
    }

    while (true)
    {
        int digital_io_read = 0;
        for (int pin_id = 0; pin_id < 4; pin_id++)
        {
            digital_io_read |= io.myDigitalRead(100 + pin_id) << pin_id;

            Serial.print(io.myDigitalRead(100 + pin_id));
        }
        Serial.println();
        Serial.println(digital_io_read);
        Serial.println(io.myDigitalRead(END_STOP_PIN));
    }
}

void blinkDynamixels(bool doTest)
{
    if (!doTest)
    {
        return;
    }

    if ((millis() / 1000) % 2)
    {
        dxl.ledOn(myAX12s[0].id);
        dxl.ledOn(myAX12s[1].id);
        dxl.ledOn(myAX12s[2].id);
        dxl.ledOn(255);
    }
    else
    {
        dxl.ledOff(myAX12s[0].id);
        dxl.ledOff(myAX12s[1].id);
        dxl.ledOff(myAX12s[2].id);
        dxl.ledOff(255);
    }
}