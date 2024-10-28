#include <LiquidCrystal_I2C.h> // https://github.com/johnrickman/LiquidCrystal_I2C


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


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

void drawLCD(int a_current_score)
{
    // Print a message to the LCD.
    lcd.setCursor(13,1);
    print_with_padding(a_current_score);
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
//ros::Subscriber<krabi_msgs::actuators> actuators_sub("actuators_msg", actuators_cb);
//ros::Publisher pub_vacuum("vacuum", &vacuum_msg);

void writeFullScreen()
{
    lcd.init();
    lcd.backlight();
    lcd.clear();
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