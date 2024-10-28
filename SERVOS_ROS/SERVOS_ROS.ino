/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

#include <ros.h>
#include <krabi_msgs/servo_cmd.h>
#include <krabi_msgs/actuators.h>
#include <krabi_msgs/vacuum_pump.h>
#include <std_msgs/Float32.h>

#include <Wire.h> 
//#include <VarSpeedServo.h> // https://github.com/netlabtoolkit/VarSpeedServo
//#include "can_structs.h"
#include "can_manager.h"
#include "servos_manager.h"
#include "LCD_manager.h"
#include "RGBStrip_manager.h"
#include "pressure_sensor_manager.h"



#define RIGHT_CLAW_SERVO 0
#define RIGHT_CLAW_SERVO_PIN 9
#define LEFT_CLAW_SERVO 1
#define LEFT_CLAW_SERVO_PIN 10
#define UNUSED_SERVO 2
#define UNUSED_SERVO_PIN 5
#define CERISE_SERVO 3
#define CERISE_SERVO_PIN 6
#define BASE_SERVO 4
#define BASE_SERVO_PIN 44
#define MID_SERVO 5
#define MID_SERVO_PIN 45
#define SUCTION_SERVO 6
#define SUCTION_SERVO_PIN 46


// 3 J5
// 5 J6
// 6 J7
// 7 J18
// 13 J19
// 9 J8
// 10 J9
// 11 J10

#define SUCTION_CUP_PIN 8
#define VALVE_PIN 2

uint8_t current_score;

PersistentServo* myPersistentServos[NB_SERVOS];
ServoMessage SERVO_1_msg;

void setup()
{ 
    Serial.begin(57600);
    Serial.setTimeout(100);
    
    LCD_setup();

    int servoIndex = 0;
    myPersistentServos[servoIndex++] = new PersistentServo(1);
    myPersistentServos[servoIndex++] = new PersistentServo(2);
    myPersistentServos[servoIndex++] = new PersistentServo(3);
    myPersistentServos[servoIndex++] = new PersistentServo(6);


    delay(10);
    
    //digitalWrite(SUCTION_CUP_PIN, HIGH);
    //digitalWrite(VALVE_PIN, HIGH);

    current_score = 0;   

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)

    turnlightsoff();

    setupCAN();
}


void loop()
{
  double pressure = 0;//readPressure();
 
    
  for (int i = 0; i < 10; i++)
  {
    drawLCD(current_score);
    for (int j = 0; j < 10; j++)
    {
        int new_score = 0;
        loopCAN(new_score, &SERVO_1_msg);
        current_score = new_score; // gnagnagna rvalue, lvalue...

        myPersistentServos[0]->angle = SERVO_1_msg.angle_s1;
        myPersistentServos[0]->speed = SERVO_1_msg.speed_s1;
        myPersistentServos[1]->angle = SERVO_1_msg.angle_s2;
        myPersistentServos[1]->speed = SERVO_1_msg.speed_s2;
        myPersistentServos[2]->angle = SERVO_1_msg.angle_s3;
        myPersistentServos[2]->speed = SERVO_1_msg.speed_s3;
        myPersistentServos[3]->angle = SERVO_1_msg.angle_s4;
        myPersistentServos[3]->speed = SERVO_1_msg.speed_s4;

    }
    delay(5);
  }
}
