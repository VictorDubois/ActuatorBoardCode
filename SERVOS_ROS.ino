/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

#include <ros.h>
#include <goal_strategy/servos_cmd.h>
#include <VarSpeedServo.h>

#define BRAK 0
#define BRAK_PIN 10//Todo check
#define PAVILLON 1
#define PAVILLON_PIN 11//Todo check
#define NB_SERVOS 2

ros::NodeHandle nh;
VarSpeedServo myServos[NB_SERVOS];
bool stopped = true;

void cmd_servos_cb(const goal_strategy::servos_cmd& command)
{
    if (!command.enable) {
        if (!stopped) {
            for( int i = 0; i< NB_SERVOS; i++ ) {
                myServos[i].detach();
            }
        }
        stopped = true;
        return;
    }
    if(stopped)
    {
        stopped = false;
        myServos[BRAK].attach(BRAK_PIN);
        myServos[PAVILLON].attach(PAVILLON_PIN);
    }
    myServos[BRAK].write(command.brak_speed, command.brak_angle, false);
    myServos[PAVILLON].write(command.pavillon_speed, command.pavillon_angle, false);
}

ros::Subscriber<goal_strategy::servos_cmd> servos_cmd_sub("cmd_servos", cmd_servos_cb);

void setup()
{ 
    pinMode(BRAK_PIN, OUTPUT);
    pinMode(PAVILLON_PIN, OUTPUT);
    
    nh.initNode();
    nh.subscribe(servos_cmd_sub);
}

void loop()
{
   nh.spinOnce();
   delay(50);
}

