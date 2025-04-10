/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

/*#include <ros.h>
#include <krabi_msgs/servo_cmd.h>
#include <krabi_msgs/actuators.h>
#include <krabi_msgs/vacuum_pump.h>
#include <std_msgs/Float32.h>
*/

#define PAMI

#include <Wire.h> 
//#include <VarSpeedServo.h> // https://github.com/netlabtoolkit/VarSpeedServo
#include "CanStruct/can_structs.h"
using namespace CAN;

#ifndef PAMI
  #include "can_manager.h"
#endif

#ifdef PAMI
  #include "diff_drive_steppers.h"
#endif

#include "servos_manager.h"
#include "LCD_manager.h"
#include "RGBStrip_manager.h"
#include "pressure_sensor_manager.h"
#include "stepper_manager.h"
#include "VL53L0X.h"
#include "io_expender_manager.h"

uint8_t current_score;

PersistentServo* myPersistentServos[NB_SERVOS];
IOPotentiallyExpended io;
VL53L0X_RangingMeasurementData_t measureFront;
VL53L0X_RangingMeasurementData_t measureRear;

#define STEPPER_POWER_ENABLE_PIN 5
#define SERVO_POWER_ENABLE_PIN 44
#define TRANSISTOR_1_PIN 104
#define TRANSISTOR_2_PIN 9
#define TRANSISTOR_3_PIN 105
#define TRANSISTOR_4_PIN 7
#define BAT_12V_PIN 1

void setup()
{ 
    Serial.begin(115200);
    Serial.setTimeout(100);


    //if (!io_exp.begin(LCM2004A_I2C_ADR1)) io_exp.begin(LCM2004A_I2C_ADR2);
    
    LCD_setup();

    int servoIndex = 0;
    myPersistentServos[servoIndex++] = new PersistentServo(35);
    myPersistentServos[servoIndex++] = new PersistentServo(36);
    myPersistentServos[servoIndex++] = new PersistentServo(37);
    myPersistentServos[servoIndex++] = new PersistentServo(39);
    myPersistentServos[servoIndex++] = new PersistentServo(40);
    myPersistentServos[servoIndex++] = new PersistentServo(41);
    myPersistentServos[servoIndex++] = new PersistentServo(42);
    myPersistentServos[servoIndex++] = new PersistentServo(43);


    delay(10);
    Serial.println("before setupAccelStep");

    setupAccelStep();
    Serial.println("after setupAccelStep");

    //digitalWrite(SUCTION_CUP_PIN, HIGH);
    //digitalWrite(VALVE_PIN, HIGH);

    current_score = 0;   

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)

    turnlightsoff();
#ifndef PAMI
    setupCAN();
#endif

    setupVL53L0X(&io);
    io.myPinMode(100, INPUT);
    io.myPinMode(101, INPUT);
    io.myPinMode(102, INPUT);
    io.myPinMode(103, INPUT);
    io.myPinMode(104, OUTPUT);
    io.myPinMode(105, OUTPUT);
    io.myPinMode(106, OUTPUT);
    io.myPinMode(107, OUTPUT);

    Serial.println("end of Setup");


    // Homing
    Stepper stepperStructHoming;
    stepperStructHoming.current = 200; //*50mA
    stepperStructHoming.accel = 100; //mm/s2
    stepperStructHoming.speed = 10; //mm/s
    stepperStructHoming.position = -1000; //mm
    stepperStructHoming.mode = stepper_mode::HOMING;
    loopStepper(stepperStructHoming);

}


void loop()
{  
  double pressure = 0;//readPressure();
  ServoMessage SERVO_1_msg;
  ServoMessage SERVO_2_msg;
  Stepper stepperStruct;
  StepperInfo stepper_info;

  uint16_t bat_12V_voltage;
  uint8_t digital_io_read;
  uint16_t digital_io_output;
  uint8_t enables;
    
  for (int i = 0; i < 10; i++)
  {
    drawLCD(current_score);

    read_dual_sensors(&io, &measureFront, &measureRear);
    bat_12V_voltage = analogRead(BAT_12V_PIN) * 5.2 * 3300/4096;

    // Digital inputs
    digital_io_read = 0;
    for (int pin_id = 0; pin_id < 4; pin_id++)
    {
        digital_io_read |= io.myDigitalRead(100+pin_id) << pin_id;
    }

    // Transistors
    io.myDigitalWrite(TRANSISTOR_1_PIN, digital_io_output & (0x1 << 0));
    io.myDigitalWrite(TRANSISTOR_2_PIN, digital_io_output & (0x1 << 1));
    io.myDigitalWrite(TRANSISTOR_3_PIN, digital_io_output & (0x1 << 2));
    io.myDigitalWrite(TRANSISTOR_4_PIN, digital_io_output & (0x1 << 3));

    io.myDigitalWrite(SERVO_POWER_ENABLE_PIN, enables & (0x1 << 0));
    //io.myDigitalWrite(STEPPER_POWER_ENABLE_PIN, enables & (0x1 << 1));

    for (int j = 0; j < 1; j++)
    {
      #ifndef PAMI
        loopCAN(current_score, &SERVO_1_msg, &SERVO_2_msg, &stepperStruct, &stepper_info, bat_12V_voltage, digital_io_read, digital_io_output, enables);
      #endif
        

        stepperStruct.current = 200; //*50mA
        stepperStruct.accel = 200; //mm/s2
        stepperStruct.speed = 200; //mm/s
        stepperStruct.position = 50; //mm
        if ((millis()/1000)%2)
        {
          stepperStruct.position = 60; //mm
        }
        stepperStruct.mode = stepper_mode::POSITION;

        for(int i = 0; i < 10; i++)
        {
          stepper_info = loopStepper(stepperStruct);
        }
        

        //Serial.println(stepper_info.distance_to_go);

        myPersistentServos[0]->angle = SERVO_1_msg.angle_s1;
        myPersistentServos[0]->speed = SERVO_1_msg.speed_s1;
        myPersistentServos[1]->angle = SERVO_1_msg.angle_s2;
        myPersistentServos[1]->speed = SERVO_1_msg.speed_s2;
        myPersistentServos[2]->angle = SERVO_1_msg.angle_s3;
        myPersistentServos[2]->speed = SERVO_1_msg.speed_s3;
        myPersistentServos[3]->angle = SERVO_1_msg.angle_s4;
        myPersistentServos[3]->speed = SERVO_1_msg.speed_s4;
        
        myPersistentServos[4]->angle = SERVO_2_msg.angle_s1;
        myPersistentServos[4]->speed = SERVO_2_msg.speed_s1;
        myPersistentServos[5]->angle = SERVO_2_msg.angle_s2;
        myPersistentServos[5]->speed = SERVO_2_msg.speed_s2;
        myPersistentServos[6]->angle = SERVO_2_msg.angle_s3;
        myPersistentServos[6]->speed = SERVO_2_msg.speed_s3;
        myPersistentServos[7]->angle = SERVO_2_msg.angle_s4;
        myPersistentServos[7]->speed = SERVO_2_msg.speed_s4;

        updateServos(myPersistentServos);
    }
    //delay(5);
  }
  Serial.println(stepper_info.distance_to_go);
  //Serial.println(myPersistentServos[0]->angle);
  //Serial.println(current_score);
}
