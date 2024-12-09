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
#include "stepper_manager.h"


uint8_t current_score;

PersistentServo* myPersistentServos[NB_SERVOS];




void setup()
{ 
    Serial.begin(115200);
    Serial.setTimeout(100);
    
    LCD_setup();

    int servoIndex = 0;
    myPersistentServos[servoIndex++] = new PersistentServo(12);
    myPersistentServos[servoIndex++] = new PersistentServo(14);
    myPersistentServos[servoIndex++] = new PersistentServo(32);
    myPersistentServos[servoIndex++] = new PersistentServo(22);


    delay(10);

    setupAccelStep();
    
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
  ServoMessage SERVO_1_msg;
  ServoMessage SERVO_2_msg;
  Stepper stepperStruct;
  StepperInfo stepper_info;
    
  for (int i = 0; i < 10; i++)
  {
    drawLCD(current_score);
    for (int j = 0; j < 10; j++)
    {
        loopCAN(current_score, &SERVO_1_msg, &SERVO_2_msg, &stepperStruct, &stepper_info);
        stepper_info = loopStepper(stepperStruct);

        myPersistentServos[0]->angle = SERVO_1_msg.angle_s1;
        myPersistentServos[0]->speed = SERVO_1_msg.speed_s1;
        myPersistentServos[1]->angle = SERVO_1_msg.angle_s2;
        myPersistentServos[1]->speed = SERVO_1_msg.speed_s2;
        myPersistentServos[2]->angle = SERVO_1_msg.angle_s3;
        myPersistentServos[2]->speed = SERVO_1_msg.speed_s3;
        myPersistentServos[3]->angle = SERVO_1_msg.angle_s4;
        myPersistentServos[3]->speed = SERVO_1_msg.speed_s4;
        
        /*myPersistentServos[4]->angle = SERVO_2_msg.angle_s1;
        myPersistentServos[4]->speed = SERVO_2_msg.speed_s1;
        myPersistentServos[5]->angle = SERVO_2_msg.angle_s2;
        myPersistentServos[5]->speed = SERVO_2_msg.speed_s2;
        myPersistentServos[6]->angle = SERVO_2_msg.angle_s3;
        myPersistentServos[6]->speed = SERVO_2_msg.speed_s3;
        myPersistentServos[7]->angle = SERVO_2_msg.angle_s4;
        myPersistentServos[7]->speed = SERVO_2_msg.speed_s4;*/

        updateServos(myPersistentServos);
    }
    delay(5);
  }
  Serial.println(myPersistentServos[0]->angle);
  Serial.println(current_score);
}
