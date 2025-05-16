/****************************************************************************
 * Author : Victor Dubois
 ****************************************************************************/

// #define PAMI 1

#include <Wire.h>
#include "CanStruct/can_structs.h"
using namespace CAN;

#ifndef PAMI // actuator board
#include "can_manager.h"
#include "stepper_manager.h"
#include "AX12_manager.h"
#endif

#ifdef PAMI
#include "diff_drive_steppers.h"
#include "MPU6050.h"
#include "VL53L0X.h"
#endif

#include "oled_manager.h"
#include "servos_manager.h"
#include "RGBStrip_manager.h"
#include "pressure_sensor_manager.h"
#include "io_expender_manager.h"

uint8_t current_score;

PersistentServo *myPersistentServos[NB_SERVOS];
IOPotentiallyExpended io;

uint32_t endOfMatch;
int8_t teamIsBlue = -1;
int8_t remaining_time_s = 127;
uint32_t timestamp_start_match = 0;

#define STEPPER_POWER_ENABLE_PIN 5
#define SERVO_POWER_ENABLE_PIN 44
#define TRANSISTOR_1_PIN 104
#define TRANSISTOR_2_PIN 9
#define TRANSISTOR_3_PIN 105
#define TRANSISTOR_4_PIN 7
#define BAT_12V_PIN 1
#define TIRETTE_PAMI_PIN 100
#define TEAM_COLOR_PAMI_PIN 101
#define CAN_DETECT_0 100
#define CAN_DETECT_1 101
#define CAN_DETECT_2 102
#define CAN_DETECT_3 103

#define I2C_EXTRA_1 106
#define I2C_EXTRA_2 107

#define I2C_SDA 13
#define I2C_SCL 14

void initI2C()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

#ifdef __AVR__
  Wire.setWireTimeout(300 /* us */, true /* reset_on_timeout */);
#else
  Wire.setTimeOut(300); // ms // Todo: test this on ESP32
#endif
}

#ifndef PAMI // Actuator board
void setup()
{
  // For the PCB with the broken DCDC
  // pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  // digitalWrite(SERVO_POWER_ENABLE_PIN, LOW);
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("Start Actuator board");

  initI2C();

  io.begin();

  // IO expender
  io.myPinMode(CAN_DETECT_0, INPUT);
  io.myPinMode(CAN_DETECT_1, INPUT);
  io.myPinMode(CAN_DETECT_2, INPUT);
  io.myPinMode(CAN_DETECT_3, INPUT);
  io.myPinMode(TRANSISTOR_1_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_3_PIN, OUTPUT);
  io.myPinMode(I2C_EXTRA_1, OUTPUT);
  io.myPinMode(I2C_EXTRA_2, OUTPUT);

  // Regular IO
  io.myPinMode(STEPPER_POWER_ENABLE_PIN, OUTPUT);
  io.myPinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_2_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_4_PIN, OUTPUT);
  io.myPinMode(BAT_12V_PIN, INPUT);

  setupDynamixel();

  // Test dynamixel
  while (false)
  {
    dxl.ledOn(2);
    Serial.println("AX12 LED on");
    delay(1000);
    Serial.println("1000ms");

    dxl.ledOff(1);
    Serial.println("AX12 LED off");
    delay(1000);
  }

  setupOled();

  int servoIndex = 0;
  myPersistentServos[servoIndex++] = new PersistentServo(35);
  myPersistentServos[servoIndex++] = new PersistentServo(36);
  myPersistentServos[servoIndex++] = new PersistentServo(37);
  myPersistentServos[servoIndex++] = new PersistentServo(39);
  myPersistentServos[servoIndex++] = new PersistentServo(40);
  myPersistentServos[servoIndex++] = new PersistentServo(41);
  myPersistentServos[servoIndex++] = new PersistentServo(42);
  // myPersistentServos[servoIndex++] = new PersistentServo(43); // Clicou instead

  // test servos
  if (false)
  {
    digitalWrite(SERVO_POWER_ENABLE_PIN, LOW);
    myPersistentServos[0]->speed = 128;
    myPersistentServos[1]->speed = 128;
    myPersistentServos[2]->speed = 128;
    myPersistentServos[3]->speed = 128;
    myPersistentServos[4]->speed = 128;
    myPersistentServos[5]->speed = 128;
    myPersistentServos[6]->speed = 128;
    myPersistentServos[7]->speed = 128;

    while (true)
    {
      myPersistentServos[0]->angle = 128;
      myPersistentServos[1]->angle = 128;
      myPersistentServos[2]->angle = 128;
      myPersistentServos[3]->angle = 128;
      myPersistentServos[4]->angle = 128;
      myPersistentServos[5]->angle = 128;
      myPersistentServos[6]->angle = 128;
      myPersistentServos[7]->angle = 128;

      updateServos(myPersistentServos);
      delay(1000);

      myPersistentServos[0]->angle = 128;
      myPersistentServos[1]->angle = 128;
      myPersistentServos[2]->angle = 128;
      myPersistentServos[3]->angle = 128;
      myPersistentServos[4]->angle = 128;
      myPersistentServos[5]->angle = 128;
      myPersistentServos[6]->angle = 128;
      myPersistentServos[7]->angle = 128;

      updateServos(myPersistentServos);
      delay(1000);
    }
  }
  delay(15);
  Serial.println("Init AccelStep");
  setupAccelStep();

  // digitalWrite(SUCTION_CUP_PIN, HIGH);
  // digitalWrite(VALVE_PIN, HIGH);

  current_score = 0;

  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  turnlightsoff();

  setupCAN();
  Serial.println("Setup CAN done");

  // Test Dynamixel
  while (false)
  {
    CAN::AX12Write l_ax12_msg;
    l_ax12_msg.currentLimit = 255;
    l_ax12_msg.max_accel = 255;
    l_ax12_msg.max_speed = 255;
    l_ax12_msg.temperatureLimit = 65;
    l_ax12_msg.position = 100;
    l_ax12_msg.torque_enable = 1;

    for (int i = 0; i < 10; i++)
    {
      // myAX12s[0].commands.position(300);
      l_ax12_msg.position = 300 + 50 * i;

      updateDynamixel(l_ax12_msg, 1);
      Serial.println(l_ax12_msg.position);
      delay(1000);
    }
  }

  Serial.println("end of Setup");

  // Homing
  Stepper stepperStructHoming;
  stepperStructHoming.current = 200; //*50mA
  stepperStructHoming.accel = 200;   // mm/s2
  stepperStructHoming.speed = 100;   // mm/s
  stepperStructHoming.position = 0;  // mm
  stepperStructHoming.mode = stepper_mode::HOMING;
  // stepperStructHoming.mode = stepper_mode::POSITION;
  loopStepper(stepperStructHoming);

  // Test stepper
  if (false)
  {
    stepperStructHoming.mode = stepper_mode::POSITION;

    while (true)
    {
      stepperStructHoming.position = 0; // mm

      if ((millis() / 1000) % 2)
      {
        stepperStructHoming.position = 100; // mm
      }
      loopStepper(stepperStructHoming);
    }
  }
}
#else
void setup() // PAMI
{
  // For the PCB with the broken DCDC
  // pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  // digitalWrite(SERVO_POWER_ENABLE_PIN, LOW);
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("PAMI setup");

  initI2C();

  io.begin();

  io.myPinMode(TIRETTE_PAMI_PIN, INPUT);
  io.myPinMode(TEAM_COLOR_PAMI_PIN, INPUT);
  io.myPinMode(CAN_DETECT_2, INPUT);
  io.myPinMode(CAN_DETECT_3, INPUT);
  io.myPinMode(TRANSISTOR_1_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_3_PIN, OUTPUT);
  io.myPinMode(I2C_EXTRA_1, OUTPUT);
  io.myPinMode(I2C_EXTRA_2, OUTPUT);

  io.myPinMode(STEPPER_POWER_ENABLE_PIN, OUTPUT);
  io.myPinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_2_PIN, OUTPUT);
  io.myPinMode(TRANSISTOR_4_PIN, OUTPUT);
  io.myPinMode(BAT_12V_PIN, INPUT);

  myPersistentServos[0] = new PersistentServo(35);

  // setupVL53L0XDual(&io);
  setupVL53L0XSingle();

  setupAccelStepPami();

  setupMPU6050();

  setupOled();

  // Wait Tirette
  while (io.myDigitalRead(TIRETTE_PAMI_PIN))
  {
    delay(10);
  }
  timestamp_start_match = millis();
  uint32_t timeout_start_PAMI = timestamp_start_match + 85000;
  endOfMatch = timestamp_start_match + 100000;

  teamIsBlue = io.myDigitalRead(TEAM_COLOR_PAMI_PIN);

  showTeamColor(teamIsBlue);

  // wait for PAMI-time to come
  while (timeout_start_PAMI > millis())
  {
    loopOled((millis() - timestamp_start_match) / 1000, teamIsBlue);
    delay(10);
  }

  endOfMatch = millis() + 15000;
  Serial.println("PAMI GO !");
}

#endif

#ifdef PAMI
void funnyAction()
{
  myPersistentServos[0]->angle = 45;
  if (millis() / 1000 % 2)
  {
    myPersistentServos[0]->angle = 100;
  }
  myPersistentServos[0]->update();
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measureFront;
  VL53L0X_RangingMeasurementData_t measureRear;
  bool tiltedRight = false;
  bool tiltedLeft = false;
  bool tiltedFront = false;
  // read_dual_sensors(&io, &measureFront, &measureRear);
  read_single_sensor(&io, &measureFront);
  bool stop = obstaclePresent(&measureFront, 100);
  updateTiltedStatus(tiltedRight, tiltedLeft, tiltedFront);

  loopOled((millis() - timestamp_start_match) / 1000, teamIsBlue);

  if (millis() > endOfMatch)
  {
    funnyAction();
    stop = true;
    Serial.println("End of match!");
    disableSteppers();
    delay(1000);
  }
  for (int i = 0; i < 10; i++) // run loopStepper more often than slow I2C calls
  {
    loopStepperPami(stop, tiltedLeft, tiltedRight, teamIsBlue, tiltedFront);
  }
}
#else // not PAMI
void loop()
{
  double pressure = 0; // readPressure();
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
    // drawLCD(current_score);
    // current_score = (millis()/100)%256;
    loopOled(current_score, remaining_time_s, teamIsBlue);
    showTeamColor(teamIsBlue);

    updateDynamixels();
    updateDynamixelsInfo();

    // read_dual_sensors(&io, &measureFront, &measureRear);
    bat_12V_voltage = analogRead(BAT_12V_PIN) * 5.2 * 3300 / 4096;

    // Digital inputs
    digital_io_read = 0;
    for (int pin_id = 0; pin_id < 4; pin_id++)
    {
      digital_io_read |= io.myDigitalRead(100 + pin_id) << pin_id;
    }

    // Transistors
    io.myDigitalWrite(TRANSISTOR_1_PIN, digital_io_output & (0x1 << 0));
    io.myDigitalWrite(TRANSISTOR_2_PIN, digital_io_output & (0x1 << 1));
    io.myDigitalWrite(TRANSISTOR_3_PIN, digital_io_output & (0x1 << 2));
    io.myDigitalWrite(TRANSISTOR_4_PIN, digital_io_output & (0x1 << 3));

    io.myDigitalWrite(SERVO_POWER_ENABLE_PIN, enables & (0x1 << 0));
    // io.myDigitalWrite(STEPPER_POWER_ENABLE_PIN, enables & (0x1 << 1));

    for (int j = 0; j < 1; j++) // run loopCAN more often than slow I2C calls
    {
      // Serial.println("loopCan");
      loopCAN(current_score, &SERVO_1_msg, &SERVO_2_msg, &stepperStruct, &stepper_info, bat_12V_voltage, digital_io_read, digital_io_output, enables, remaining_time_s, teamIsBlue,
              myAX12s[0].commands, myAX12s[1].commands, myAX12s[2].commands, myAX12s[3].commands, myAX12s[4].commands, myAX12s[5].commands,
              myAX12s[0].infos, myAX12s[1].infos, myAX12s[2].infos, myAX12s[3].infos, myAX12s[4].infos, myAX12s[5].infos);

      for (int i = 0; i < 10; i++) // run loopStepper more often than slow stuff
      {
        stepper_info = loopStepper(stepperStruct);
      }

      // Serial.println(stepper_info.distance_to_go);

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
      // myPersistentServos[7]->angle = SERVO_2_msg.angle_s4;
      // myPersistentServos[7]->speed = SERVO_2_msg.speed_s4;

      updateServos(myPersistentServos);
    }
    // delay(5);
  }
  // Serial.println(stepper_info.distance_to_go);
  // Serial.println(myPersistentServos[0]->angle);
  // Serial.println(current_score);
}
#endif
