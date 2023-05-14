/****************************************************************************
* Author : Victor Dubois
****************************************************************************/

#include <ros.h>
#include <krabi_msgs/servo_cmd.h>
#include <krabi_msgs/actuators.h>
#include <krabi_msgs/vacuum_pump.h>
#include <std_msgs/Float32.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h> // https://github.com/johnrickman/LiquidCrystal_I2C
#include <VarSpeedServo.h> // https://github.com/netlabtoolkit/VarSpeedServo

////// NeoPixel
// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LED_PIN 4
#define LED_COUNT 80
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
unsigned long pixelPrevious = 0;        // Previous Pixel Millis
unsigned long patternPrevious = 0;      // Previous Pattern Millis
int           patternCurrent = 0;       // Current Pattern Number
int           patternInterval = 5000;   // Pattern Interval (ms)
int           pixelInterval = 50;       // Pixel Interval (ms)
int           pixelQueue = 0;           // Pattern Pixel Queue
int           pixelCycle = 0;           // Pattern Pixel Cycle
uint16_t      pixelCurrent = 0;         // Pattern Current Pixel Number
uint16_t      pixelNumber = LED_COUNT;  // Total Number of Pixels
// end NeoPixel

#define BASE_SERVO 0
#define BASE_SERVO_PIN 9
#define MID_SERVO 1
#define MID_SERVO_PIN 10
#define SUCTION_SERVO 2
#define SUCTION_SERVO_PIN 5
#define PUSHER_SERVO 3
#define PUSHER_SERVO_PIN 6
#define NB_SERVOS 4

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

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

ros::NodeHandle nh;
bool stopped = true;
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
uint8_t current_score;
krabi_msgs::actuators persistent_actuators_command;
std_msgs::Float32 vacuum_msg;
int16_t sent_servos_angles[NB_SERVOS];
uint8_t servo_pins[NB_SERVOS];
bool stopped_servos_last_update[NB_SERVOS];
VarSpeedServo myservos[NB_SERVOS];
bool disguise = false;
bool disguise_done = false;

/*
pressure_range (kPa)   K value
131<P≤260               32
65<P≤131                64
32<P≤65                 128
16<P≤32                 256
8<P≤16                  512
4<P≤8                   1024
2≤P≤4                   2048
1≤P<2                   4096
P<1                     8192
*/

#define K 512 // @TODO see table above to choose the correct value for your application
#define I2C_address 0x6D // Unfortunately, it does not seem possible to change it. To use multiple sensors, use the analog version (XGZP6897A)


double readPressure()
{
    unsigned char pressure_H, pressure_M, pressure_L, temperature_H, temperature_L;
    //temporary variables of pressure and temperature
    long int pressure_adc, temperature_adc;
    //The value of pressure and temperature converted by the sensor’s ADC
    double pressure, temperature;
    //The calibrated value of pressure 

    write_one_byte(I2C_address, 0x30, 0x0A);

        
    //indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)
    //more measurement method, check Register 0x30
    
    long timeout = 0;
    while ((Read_One_Byte(I2C_address, 0x30) & 0x08) > 0)// && timeout < 65000)
    {
      timeout++;
      if (timeout > 5000)
      {
        //Serial.println("Timeout! Please check the connections");
        break;
      }
    }
    //Judge whether Data collection is over
    delay(20);
    
    pressure_H = Read_One_Byte(I2C_address, 0x06);
    pressure_M = Read_One_Byte(I2C_address, 0x07);
    pressure_L = Read_One_Byte(I2C_address, 0x08);
    
    // Read ADC output Data of Pressure
    pressure_adc = pressure_H * 65536 + pressure_M * 256 + pressure_L;
    //Compute the value of pressure converted by ADC
    if (pressure_adc > 8388608)
    {
      pressure = (pressure_adc - 16777216) / K; //unit is Pa, select appropriate K value according to pressure range. 
    }
    else
    {
      pressure = pressure_adc / K; //unit is Pa, select appropriate K value according to pressure range. //The conversion formula of calibrated pressure, its unit is Pa
    }

    // Debug, feel free to comment this out
    /*Serial.print("Pressure: ");
    Serial.print(int(pressure));
    Serial.print(", Temperature: ");
    Serial.println(int(temperature));*/
    
    temperature_H = Read_One_Byte(I2C_address, 0x09);
    temperature_L = Read_One_Byte(I2C_address, 0x0A);
    //Read ADC output data of temperature
    temperature_adc = temperature_H * 256 + temperature_L;
    //Compute the value of temperature converted by ADC
    temperature = (temperature_adc - 65536)/256; // is deg C
    //else // I do not understand this else without an "if" in the documentation
    temperature = temperature_adc / 256; //unit is ℃
    
    //The conversion formula of calibrated temperature, its unit is Centigrade
    
    return pressure;
}

//----write One Byte of Data, Data from Arduino to the sensor----
// Write "thedata" to the sensor's address of "addr"
void write_one_byte(uint8_t device_address, uint8_t addr, uint8_t thedata)
{
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.write(thedata);
  Wire.endTransmission();
}

//----Read One Byte of Data, Data from the sensor to the Arduino ---- 
uint8_t Read_One_Byte(uint8_t device_address, uint8_t addr)
{
  uint8_t nb_bytes = 1;
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(device_address, nb_bytes); 
  return Wire.read();    // Receive a byte as character    
}

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
    //pwm.setPWM(servo_pins[servo_id], 0, map(sent_servos_angles[servo_id], 0, 255, SERVOMIN, SERVOMAX));
    myservos[servo_id].write(sent_servos_angles[servo_id]);
    
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
    write_servo_cmd_from_actuator(BASE_SERVO, persistent_actuators_command.arm_base_servo);
    write_servo_cmd_from_actuator(MID_SERVO, persistent_actuators_command.arm_mid_servo);
    write_servo_cmd_from_actuator(SUCTION_SERVO, persistent_actuators_command.arm_suction_cup_servo);
    write_servo_cmd_from_actuator(PUSHER_SERVO, persistent_actuators_command.pusher_servo);

    digitalWrite(SUCTION_CUP_PIN, persistent_actuators_command.arm_vacuum.enable_pump);
    digitalWrite(VALVE_PIN, persistent_actuators_command.arm_vacuum.release);

    disguise = persistent_actuators_command.fake_statuette_vacuum.enable_pump;// Hack to avoid redefining a msg
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
ros::Subscriber<krabi_msgs::actuators> actuators_sub("actuators_msg", actuators_cb);
ros::Publisher pub_vacuum("vacuum", &vacuum_msg);

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

void setup()
{ 
    Serial.begin(57600);
    Wire.begin();
    Wire.setClock(100000);


    delay(10);

    nh.initNode();
    nh.subscribe(actuators_sub);
    nh.advertise(pub_vacuum);
    nh.spinOnce();


    lcd.init();                      // initialize the lcd 
    lcd.backlight();
    createCrab();
    writeFullScreen();
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
    persistent_actuators_command.fake_statuette_vacuum.release = false;

    servo_pins[BASE_SERVO] = BASE_SERVO_PIN;
    servo_pins[MID_SERVO] = MID_SERVO_PIN;
    servo_pins[SUCTION_SERVO] = SUCTION_SERVO_PIN;
    servo_pins[PUSHER_SERVO] = PUSHER_SERVO_PIN;

    for (int i = 0; i < NB_SERVOS ; i++)
    {
        myservos[i].attach(servo_pins[i]);
    }

    delay(10);
    pinMode(BASE_SERVO_PIN, OUTPUT);
    pinMode(MID_SERVO_PIN, OUTPUT);
    pinMode(SUCTION_SERVO_PIN, OUTPUT);
    pinMode(PUSHER_SERVO_PIN, OUTPUT);
    pinMode(SUCTION_CUP_PIN, OUTPUT);
    pinMode(VALVE_PIN, OUTPUT);
    
    //digitalWrite(SUCTION_CUP_PIN, HIGH);
    //digitalWrite(VALVE_PIN, HIGH);

    current_score = 0;   
    disguise_done = false; 

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)

    turnlightsoff();

}

void lightUpAll()
{
    strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
    strip.fill(strip.Color(255, 255, 255), 0, LED_COUNT);
    strip.show();
}

void turnlightsoff()
{
    strip.setBrightness(10); // Set BRIGHTNESS to about 1/5 (max = 255)
    strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT);
    strip.show();
}

void loop()
{
  writeFullScreen();
  double pressure = readPressure();
  vacuum_msg.data = pressure;
  pub_vacuum.publish(&vacuum_msg);
  if (disguise && !disguise_done)
  {
    lightUpAll();
    disguise_done = true;
  }
    
  for (int i = 0; i < 10; i++)
  {
    drawLCD();
    for (int j = 0; j < 10; j++)
    {
        nh.spinOnce();
        update_actuators();
    }
    delay(5);
  }
}
