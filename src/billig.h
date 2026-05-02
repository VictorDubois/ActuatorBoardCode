#include "AX12_manager.h"
#include "servos_manager.h"

#include "constants.h"

void initBillig(PersistentServo *myPersistentServos[])
{
    // Init AX12 positions to closed
    Serial.println("Init AX12 positions to closed");
    {
        // while (true)
        {
            for (int i = 0; i < NB_AX12; i++)
            {
                myAX12s[i].commands.position = 0;
                myAX12s[i].commands.currentLimit = 100;
                myAX12s[i].commands.max_accel = 100;
                myAX12s[i].commands.max_speed = 100;
                myAX12s[i].commands.torque_enable = 1;

                updateDynamixel(myAX12s[i].commands, myAX12s[i].id);

                dxl.ledOn(myAX12s[i].id);
                Serial.print("AX12 ID: ");
                Serial.println(myAX12s[i].id);

                delay(100);
                dxl.ledOff(myAX12s[i].id);
            }
        }
        // delay(2000);
    }

    Serial.println("Turn the servos to the default position");

    // Init servos to default position
    {
        digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);

        myPersistentServos[0]->angle = 30;  // 0 = endroit tout à droite
        myPersistentServos[1]->angle = 0;   // 0 = endroit milieu droite
        myPersistentServos[2]->angle = 0;   // 0 = endroit milieu gauche
        myPersistentServos[3]->angle = 0;   // 0 = endroit tout à gauche
        myPersistentServos[4]->angle = 175; // 175 = doigt en bas
        myPersistentServos[5]->angle = 150;
        myPersistentServos[6]->angle = 150;

        for (int l_i = 0; l_i < 100; l_i++)
        {
            updateServos(myPersistentServos);
            delay(30);
        }
    }

    Serial.println("close AX12s");
    // Init AX12 positions to open
    {
        for (int i = 0; i < NB_AX12; i++)
        {
            myAX12s[i].commands.position = 800;
            myAX12s[i].commands.currentLimit = 50;
            myAX12s[i].commands.max_accel = 100;
            myAX12s[i].commands.max_speed = 100;
            myAX12s[i].commands.torque_enable = 1;
            updateDynamixel(myAX12s[i].commands, myAX12s[i].id);
        }
    }
    Serial.println("Grabbers init done");
}
