/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

#include <ros.h>
#include <krabi_msgs/servos_cmd.h>
#include <VarSpeedServo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define BRAK 0
#define BRAK_PIN 9
#define PAVILLON 1
#define PAVILLON_PIN 10
#define TAPETTE_PHARE 2
#define TAPETTE_PHARE_PIN 11
#define NB_SERVOS 3

ros::NodeHandle nh;
VarSpeedServo myServos[NB_SERVOS];
bool stopped = true;
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
uint8_t current_score;

void cmd_servos_cb(const krabi_msgs::servos_cmd& command)
{
    current_score = command.s4_speed;// hack to store the score
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
        myServos[TAPETTE_PHARE].attach(TAPETTE_PHARE_PIN);
    }

    myServos[BRAK].write(command.brak_angle, command.brak_speed, false);
    myServos[PAVILLON].write(command.pavillon_angle, command.pavillon_speed, false);
    myServos[TAPETTE_PHARE].write(command.s3_angle, command.s3_speed, false);
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
ros::Subscriber<krabi_msgs::servos_cmd> servos_cmd_sub("cmd_servos", cmd_servos_cb);

void setup()
{ 
    pinMode(BRAK_PIN, OUTPUT);
    pinMode(PAVILLON_PIN, OUTPUT);
    pinMode(TAPETTE_PHARE_PIN, OUTPUT);
    
    nh.initNode();
    nh.subscribe(servos_cmd_sub);
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
    nh.spinOnce();
    drawLCD();
    delay(50);
}
