/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

#include <ros.h>
#include <krabi_msgs/servo_cmd.h>
#include <krabi_msgs/actuators.h>
#include <krabi_msgs/vacuum_pump.h>

#include <VarSpeedServo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PWMServoDriver.h>

#define BASE_SERVO 0
#define BASE_SERVO_PIN 9
#define MID_SERVO 1
#define MID_SERVO_PIN 10
#define SUCTION_SERVO 2
#define SUCTION_SERVO_PIN 11
#define PUSHER_SERVO 2
#define PUSHER_SERVO_PIN 3
#define NB_SERVOS 4

#define SUCTION_CUP_PIN 5
#define VALVE_PIN 6

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle nh;
VarSpeedServo myServos[NB_SERVOS];
bool stopped = true;
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
uint8_t current_score;
krabi_msgs::actuators persistent_actuators_command;
int16_t sent_servos_angles[NB_SERVOS];
uint8_t servo_pins[NB_SERVOS];
bool stopped_servos_last_update[NB_SERVOS];

void write_servo_cmd(uint8_t servo_id, int16_t servo_cmd_angle, int16_t servo_cmd_speed)
{
    if (servo_cmd_angle == sent_servos_angles[servo_id])
    {
      return;
    }
    delay(50);
    if (servo_cmd_angle > sent_servos_angles[servo_id] + servo_cmd_speed)
    {
        sent_servos_angles[servo_id] += servo_cmd_speed;
    }
    else if (servo_cmd_angle < sent_servos_angles[servo_id] - servo_cmd_speed)
    {
        sent_servos_angles[servo_id] -= servo_cmd_speed;
    }
    else
    {
        sent_servos_angles[servo_id] = servo_cmd_angle;
    }
    pwm.setPWM(servo_pins[servo_id], 0, map(sent_servos_angles[servo_id], 0, 255, SERVOMIN, SERVOMAX));
    //myServos[servo_id].write(servo_cmd_angle, servo_cmd_speed, false);
}


void actuators_cb(const krabi_msgs::actuators& command)
{
    persistent_actuators_command = command;
    current_score = command.score;
}

void write_servo_cmd_from_actuator(uint8_t servo_id, const krabi_msgs::servo_cmd& command)
{
  if (!command.enable)
  {
    if (!stopped_servos_last_update[servo_id])
    {
      myServos[servo_id].detach();
      pwm.setPin(servo_pins[servo_id], 0, true);
    }
    stopped_servos_last_update[servo_id] = true;
    return;
  }
  if(stopped_servos_last_update[servo_id])
  {
    stopped_servos_last_update[servo_id] = false;
    myServos[servo_id].attach(servo_pins[servo_id]);
  }
  write_servo_cmd(servo_id, command.angle, command.speed); 
}

void update_actuators()
{
    write_servo_cmd_from_actuator(BASE_SERVO, persistent_actuators_command.arm_base_servo);
    write_servo_cmd_from_actuator(MID_SERVO, persistent_actuators_command.arm_mid_servo);
    write_servo_cmd_from_actuator(SUCTION_SERVO, persistent_actuators_command.arm_suction_cup_servo);
    write_servo_cmd_from_actuator(PUSHER_SERVO, persistent_actuators_command.pusher_servo);

    digitalWrite(SUCTION_CUP_PIN, persistent_actuators_command.arm_vacuum.enable_pump);
    digitalWrite(VALVE_PIN, persistent_actuators_command.arm_vacuum.release);
}

void drawLCD()
{
    // Print a message to the LCD.
    lcd.setCursor(13,1);
    print_with_padding(current_score);
}

void print_with_padding(uint16_t number)
{
    uint16_t hundreds = (number%1000 - number%100)/100;
    uint16_t units = number%10;
    uint16_t tens = (number%1000 - hundreds*100 - units)/10;
    if(hundreds == 0)
    {
        lcd.print(" ");
    }
    else
    {
        lcd.print(hundreds);
    };
    if(tens == 0 && hundreds == 0)
    {
        lcd.print(" ");
    }
    else
    {
        lcd.print(tens);
    };
    lcd.print(units);
}

void createCrab()
{
    byte Crab1[8] = {
        0b01000,
        0b11001,
        0b00110,
        0b01110,
        0b01110,
        0b00110,
        0b11001,
        0b01000
    };
    byte Crab2[8] = {
        0b01000,
        0b11001,
        0b00110,
        0b01111,
        0b01111,
        0b00110,
        0b11001,
        0b01000
    };
    lcd.createChar(0, Crab1);
    lcd.createChar(1, Crab2);
}
ros::Subscriber<krabi_msgs::actuators> actuators_sub("actuators", actuators_cb);

void setup()
{ 
    for(int i = 0; i < NB_SERVOS; i++) {
        sent_servos_angles[i] = 100;
        stopped_servos_last_update[i] = true;
    }

    persistent_actuators_command.arm_base_servo.enable = false;
    persistent_actuators_command.arm_mid_servo.enable = false;
    persistent_actuators_command.arm_suction_cup_servo.enable = false;
    persistent_actuators_command.pusher_servo.enable = false;
    persistent_actuators_command.arm_vacuum.enable_pump = false;
    persistent_actuators_command.arm_vacuum.release = true;
    persistent_actuators_command.fake_statuette_vacuum.enable_pump = false;
    persistent_actuators_command.fake_statuette_vacuum.release = true;

    servo_pins[BASE_SERVO] = BASE_SERVO_PIN;
    servo_pins[MID_SERVO] = MID_SERVO_PIN;
    servo_pins[SUCTION_SERVO] = SUCTION_SERVO_PIN;
    servo_pins[PUSHER_SERVO] = PUSHER_SERVO_PIN;

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    delay(10);
    pinMode(BASE_SERVO_PIN, OUTPUT);
    pinMode(MID_SERVO_PIN, OUTPUT);
    pinMode(SUCTION_SERVO_PIN, OUTPUT);
    pinMode(PUSHER_SERVO_PIN, OUTPUT);
    pinMode(SUCTION_CUP_PIN, OUTPUT);
    pinMode(VALVE_PIN, OUTPUT);
    
    digitalWrite(SUCTION_CUP_PIN, HIGH);
    digitalWrite(VALVE_PIN, HIGH);
    nh.initNode();
    nh.subscribe(actuators_sub);
    current_score = 0;
    lcd.init();                      // initialize the lcd 
    lcd.backlight();
    createCrab();
    lcd.setCursor(0,0);
    lcd.write(byte(0));
    lcd.write(byte(0));
    lcd.write(byte(0));
    
    lcd.setCursor(4,0);
    lcd.print("Kraboss");
    lcd.setCursor(13,0);
    lcd.write(byte(0));
    lcd.write(byte(1));
    lcd.write(byte(0));
    lcd.setCursor(7,1);
    lcd.print("Score:");
    
}

void loop()
{
    drawLCD();
    for (int i = 0; i < 10; i++)
    {
        nh.spinOnce();
        update_actuators();
    }
    delay(5);
}
