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

bool stopped = true;

uint8_t current_score;
krabi_msgs::actuators persistent_actuators_command;
std_msgs::Float32 vacuum_msg;
int16_t sent_servos_angles[NB_SERVOS];
uint8_t servo_pins[NB_SERVOS];
bool stopped_servos_last_update[NB_SERVOS];
Servo myservos[NB_SERVOS];
bool disguise = false;
bool disguise_done = false;



PersistentServo* myPersistentServos[NB_SERVOS];


void write_servo_cmd(uint8_t servo_id, int16_t servo_cmd_angle, int16_t servo_cmd_speed)
{
  // When easing constant (ke) < 1.0, return value is normalized, when 1.0, returns pulse width (μs)
  // ke = 0.0 is linear, between 0.0 and 1.0 is tunable sigmoid, 1.0 is normal response
  // Normalized Tunable Sigmoid: https://www.desmos.com/calculator/ejkcwglzd1
  float l_easing_constant = 0.8;
  myservos[servo_id].write(servo_pins[servo_id], servo_cmd_angle, servo_cmd_speed, l_easing_constant);    
}


void actuators_cb(const krabi_msgs::actuators& command)
{
    persistent_actuators_command = command;
    current_score = command.score;
}

float get_float(String* a_string, int a_beginning)
{
    //Serial.print((*a_string).c_str());
    String hexValue = a_string->substring(a_beginning, a_beginning+8); // Extract hexadecimal value
    //Serial.print(hexValue);
    uint32_t floatHex = strtoul(hexValue.c_str(), NULL, 16); // Convert hexadecimal string to unsigned long
    //Serial.print(floatHex);
    float floatValue;
    memcpy(&floatValue, &floatHex, sizeof(floatValue)); // Convert unsigned long to float
    return floatValue;
}

void read_serial()
{
    String line = Serial.readStringUntil('\n');
    if(line.length() < 10 + (7*3+1) *4 )
    {
      return;// Line too short, invalid
    }
    actuators_hex_cb(line);
}

void write_float(float a_value)
{
    uint32_t floatHex;
    memcpy(&floatHex, &a_value, sizeof(a_value));
    Serial.print(floatHex, HEX); 
}
void write_serial(float a_vacuum)
{
    Serial.print("From Arduino:");
    write_float(a_vacuum);
    Serial.println();
}

void actuators_hex_cb(String a_message)
{
    int i = 0;
    const int offset = 0;//10;
    const int float_msg_size = 8;
    persistent_actuators_command.arm_base_servo.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.arm_base_servo.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.arm_base_servo.angle = get_float(&a_message, offset + (i++)*float_msg_size);

    persistent_actuators_command.arm_mid_servo.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.arm_mid_servo.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.arm_mid_servo.angle = get_float(&a_message, offset + (i++)*float_msg_size);


    /*Serial.println("coucou");
    Serial.print(persistent_actuators_command.arm_base_servo.enable);
    Serial.print(persistent_actuators_command.arm_base_servo.speed);
    Serial.print(persistent_actuators_command.arm_base_servo.angle);
    Serial.print(persistent_actuators_command.arm_mid_servo.enable);
    Serial.print(persistent_actuators_command.arm_mid_servo.speed);
    Serial.println(persistent_actuators_command.arm_mid_servo.angle);*/


    persistent_actuators_command.arm_suction_cup_servo.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.arm_suction_cup_servo.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.arm_suction_cup_servo.angle = get_float(&a_message, offset + (i++)*float_msg_size);

    persistent_actuators_command.pusher_servo.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.pusher_servo.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.pusher_servo.angle = get_float(&a_message, offset + (i++)*float_msg_size);

    persistent_actuators_command.additionnal_servo_1.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.additionnal_servo_1.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.additionnal_servo_1.angle = get_float(&a_message, offset + (i++)*float_msg_size);

    persistent_actuators_command.additionnal_servo_2.enable = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.additionnal_servo_2.speed = get_float(&a_message, offset + (i++)*float_msg_size);
    persistent_actuators_command.additionnal_servo_2.angle = get_float(&a_message, offset + (i++)*float_msg_size);

    persistent_actuators_command.arm_vacuum.enable_pump = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    persistent_actuators_command.arm_vacuum.release = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;
    
    persistent_actuators_command.fake_statuette_vacuum.enable_pump = get_float(&a_message, offset + (i++)*float_msg_size) > 0.5f;

    current_score = int(get_float(&a_message, offset + (i++)*float_msg_size));

    //Serial.print("score: ");
    //Serial.println(current_score);
}

void write_servo_cmd_from_actuator(uint8_t servo_id, const krabi_msgs::servo_cmd& command)
{
  if (!command.enable)
  {
  /*  if (!stopped_servos_last_update[servo_id])
    {
      pwm.setPin(servo_pins[servo_id], 0, true);
    }*/
    stopped_servos_last_update[servo_id] = true;
    return;
  }
  if(stopped_servos_last_update[servo_id])
  {
    stopped_servos_last_update[servo_id] = false;
  }
  write_servo_cmd(servo_id, command.angle, command.speed); 
}

void update_actuators()
{
    write_servo_cmd_from_actuator(CERISE_SERVO, persistent_actuators_command.pusher_servo);
    write_servo_cmd_from_actuator(RIGHT_CLAW_SERVO, persistent_actuators_command.additionnal_servo_1);
    write_servo_cmd_from_actuator(LEFT_CLAW_SERVO, persistent_actuators_command.additionnal_servo_2);

    digitalWrite(SUCTION_CUP_PIN, persistent_actuators_command.arm_vacuum.enable_pump);
    digitalWrite(VALVE_PIN, persistent_actuators_command.arm_vacuum.release);

    disguise = persistent_actuators_command.fake_statuette_vacuum.enable_pump;// Hack to avoid redefining a msg
}


void setup()
{ 
    Serial.begin(57600);
    Serial.setTimeout(100);
    Wire.begin();
    Wire.setClock(100000);

    #ifdef __AVR__
      Wire.setWireTimeout(300 /* us */, true /* reset_on_timeout */);
    #else
      Wire.setTimeOut(3);//ms
    #endif

    delay(10);

    lcd.init();                      // initialize the lcd 
    lcd.backlight();
    createCrab();
    writeFullScreen();

    int servoIndex = 0;
    myPersistentServos[servoIndex++] = new PersistentServo(1);
    myPersistentServos[servoIndex++] = new PersistentServo(2);
    myPersistentServos[servoIndex++] = new PersistentServo(3);
    myPersistentServos[servoIndex++] = new PersistentServo(6);


    delay(10);
    
    //digitalWrite(SUCTION_CUP_PIN, HIGH);
    //digitalWrite(VALVE_PIN, HIGH);

    current_score = 0;   
    disguise_done = false; 

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)

    turnlightsoff();

    setupCAN();
}



void loop()
{
  //writeFullScreen();
  double pressure = 0;//readPressure();
  vacuum_msg.data = pressure;
  //pub_vacuum.publish(&vacuum_msg);
  //@todo pub
  write_serial((float)pressure);
  
    
  for (int i = 0; i < 10; i++)
  {
    drawLCD(current_score);
    for (int j = 0; j < 10; j++)
    {
        //nh.spinOnce();
        //read_serial();

        int new_score = 0;
        ServoMessage SERVO_1_msg;
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

        // @Todo read
        update_actuators();
        if (disguise && !disguise_done)
        {
          lightUpAll();
          disguise_done = true;
        }
    }
    delay(5);
  }
}
