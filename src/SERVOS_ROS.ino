/****************************************************************************
 * Author : Victor Dubois
 ****************************************************************************/

//  #define PAMI 1
#define SUPERSTAR 1

#include <Wire.h>
#include "CanStruct/can_structs.h"
using namespace CAN;

#ifndef PAMI // actuator board
#include "can_manager.h"
#include "stepper_manager.h"
#include "AX12_manager.h"

ServoMessage SERVO_1_msg;
ServoMessage SERVO_2_msg;
Stepper stepperStruct;
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
#define BAT_POWER_PIN 1
#define PRESSURE_1_PIN 4
#define PRESSURE_2_PIN 2
#define BAT_ELEC_PIN PRESSURE_1_PIN
#define TIRETTE_PAMI_PIN 100
#define TEAM_COLOR_PAMI_PIN 101
#define CAN_DETECT_0 100
#define CAN_DETECT_1 101
#define CAN_DETECT_2 102
#define CAN_DETECT_3 103

#define I2C_EXTRA_1 106
#define I2C_EXTRA_2 107

#define SERVO_1 35
#define SERVO_2 36
#define SERVO_3 37
#define SERVO_4 39
#define SERVO_5 40
#define SERVO_6 41
#define SERVO_7 42
#define SERVO_8 43

#define CLICOU_HOMING_PIN END_STOP_PIN // 43

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
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
  Serial.begin(115200);

  /*while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  // delay(1000);

  // Serial.setTimeout(100);
  Serial.println("Start Actuator board");
  Serial.flush();

  // Test dynamixel
  if (false)
  {
    setupDynamixel();
    dxl.torqueOn(8);
    usleep(20000);

    while (true)
    {
      dxl.ledOn(8);
      usleep(20000);

      Serial.println("AX12 LED on");
      Serial.flush();
      dxl.setGoalPosition(1, 400);
      usleep(20000);

      delay(1000);

      dxl.ledOff(8);
      usleep(20000);

      dxl.setGoalPosition(1, 800);
      usleep(20000);

      Serial.println("AX12 LED off");
      Serial.flush();
      delay(1000);
    }
  }

  initI2C();

  io.begin();
  Serial.flush();

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
  io.myPinMode(BAT_POWER_PIN, INPUT);
  io.myPinMode(BAT_ELEC_PIN, INPUT);
  io.myPinMode(PRESSURE_1_PIN, INPUT);
  io.myPinMode(PRESSURE_2_PIN, INPUT);

  io.myDigitalWrite(TRANSISTOR_1_PIN, LOW);
  io.myDigitalWrite(TRANSISTOR_2_PIN, LOW);
  io.myDigitalWrite(TRANSISTOR_3_PIN, LOW);
  io.myDigitalWrite(TRANSISTOR_4_PIN, LOW);

  SERVO_1_msg.angle_s1 = 67;
  SERVO_1_msg.angle_s2 = 52;
  SERVO_1_msg.angle_s3 = 67;
  SERVO_1_msg.angle_s4 = 62;
  SERVO_1_msg.speed_s1 = 100;
  SERVO_1_msg.speed_s2 = 100;
  SERVO_1_msg.speed_s3 = 100;
  SERVO_1_msg.speed_s4 = 100;

  SERVO_2_msg.angle_s1 = 175;
  SERVO_2_msg.speed_s1 = 100;

  Serial.println("io pinmode done");

  setupOled();

  if (false)
  {
    int AX12_ID = 8;
    setupDynamixel();

    // Test dynamixel
    while (true)
    {
      dxl.ledOn(AX12_ID);
      Serial.println("AX12 LED on");
      Serial.flush();
      delay(1000);

      dxl.ledOff(AX12_ID);
      Serial.println("AX12 LED off");
      Serial.flush();
      delay(1000);
    }
  }

  int servoIndex = 0;
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_1, 67, 100, 0.8, 67, 144);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_2, 52, 100, 0.8, 52, 132);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_3, 67, 100, 0.8, 67, 144);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_4, 62, 100, 0.8, 62, 150);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_5, 175, 100, 0.8, 100, 175);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_6, 100, 100, 0.8, 112, 175);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_7, 100, 100, 0.8, 110, 175);
  // myPersistentServos[servoIndex++] = new PersistentServo(SERVO_8); // Clicou instead
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);

  // test servos
  if (false)
  {
    digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);

    // myPersistentServos[7]->speed = 128;

    while (true)
    {
      Serial.println("128");
      myPersistentServos[0]->angle = 144; // 144 = relache tout à droite
      myPersistentServos[1]->angle = 132; // 132 = relache milieu droite
      myPersistentServos[2]->angle = 144; // 144 = relache milieu gauche
      myPersistentServos[3]->angle = 150; // 150 = relache tout à gauche
      myPersistentServos[4]->angle = 175; // 175 = doigt en bas
      myPersistentServos[5]->angle = 175;
      myPersistentServos[6]->angle = 175;
      // myPersistentServos[7]->angle = 128;

      updateServos(myPersistentServos);
      delay(1000);

      Serial.println("70");
      myPersistentServos[0]->angle = 67;  // 67 = attrape tout à droite
      myPersistentServos[1]->angle = 52;  // 52 = attrape milieu droite
      myPersistentServos[2]->angle = 67;  // 67 = attrape milieu gauche
      myPersistentServos[3]->angle = 62;  // 62 = attrape tout à gauche
      myPersistentServos[4]->angle = 115; // 110 doigt à l'horizontale
      myPersistentServos[5]->angle = 112;
      myPersistentServos[6]->angle = 110;
      // myPersistentServos[7]->angle = 128;

      // full droite 125 attrape 208 relache (testeur)
      // milieu droite 107 attrape 198 relache (testeur)
      // milieu gauche 125 attrape 210 relache (testeur)
      // full gauche 115 attrape 205 relache (testeur)
      // doigt bas 230 haut 168 (testeur)

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
  Serial.flush();

  setupDynamixel();

  // Test Dynamixel
  if (false)
  {
    int AX12_ID = 8;
    // dxl.torqueOn(1);
    while (true)
    {
      CAN::AX12Write l_ax12_msg;
      l_ax12_msg.currentLimit = 100;
      l_ax12_msg.max_accel = 100;
      l_ax12_msg.max_speed = 100;
      l_ax12_msg.temperatureLimit = 65;
      l_ax12_msg.position = 100;
      l_ax12_msg.torque_enable = 1;

      for (int i = 0; i < 10; i++)
      {
        // myAX12s[0].commands.position(300);
        l_ax12_msg.torque_enable = 1;
        l_ax12_msg.position = 300 + 50 * i;
        l_ax12_msg.currentLimit = 250;
        l_ax12_msg.max_accel = 250;
        l_ax12_msg.max_speed = 250;
        if (i % 2)
        {
          dxl.ledOn(AX12_ID);
        }
        else
        {
          dxl.ledOff(AX12_ID);
        }
        // dxl.setGoalPosition(1, l_ax12_msg.position);

        updateDynamixel(l_ax12_msg, AX12_ID);
        Serial.println(l_ax12_msg.position);
        Serial.flush();

        delay(1000);
      }
    }
  }

  Serial.println("end of Setup");
  Serial.flush();

  // Test IOs
  while (false)
  {
    int digital_io_read = 0;
    for (int pin_id = 0; pin_id < 4; pin_id++)
    {
      digital_io_read |= io.myDigitalRead(100 + pin_id) << pin_id;

      Serial.print(io.myDigitalRead(100 + pin_id));
    }
    Serial.println();
    Serial.println(digital_io_read);
    Serial.println(io.myDigitalRead(END_STOP_PIN));
  }

  // Homing
  Stepper stepperStructHoming;
  stepperStructHoming.current = 200; //*50mA
  stepperStructHoming.accel = 100;   // mm/s2
  stepperStructHoming.speed = 30;    // mm/s
  stepperStructHoming.position = 0;  // mm
  stepperStructHoming.mode = stepper_mode::HOMING;
  // stepperStructHoming.mode = stepper_mode::POSITION;
  loopStepper(stepperStructHoming);

  // Do not move
  stepperStruct.mode = stepper_mode::POSITION;
  stepperStruct.position = 0;

  // Test pumps
  while (false)
  {
    io.myDigitalWrite(TRANSISTOR_1_PIN, HIGH);
    io.myDigitalWrite(TRANSISTOR_2_PIN, HIGH);
    io.myDigitalWrite(TRANSISTOR_3_PIN, HIGH);
    io.myDigitalWrite(TRANSISTOR_4_PIN, HIGH);
    Serial.println("HIGH");

    delay(1000);

    io.myDigitalWrite(TRANSISTOR_1_PIN, LOW);
    io.myDigitalWrite(TRANSISTOR_2_PIN, LOW);
    io.myDigitalWrite(TRANSISTOR_3_PIN, LOW);
    io.myDigitalWrite(TRANSISTOR_4_PIN, LOW);
    Serial.println("LOW");

    delay(2000);
  }

  // Test stepper
  if (false)
  {
    stepperStructHoming.mode = stepper_mode::POSITION;

    while (true)
    {
      stepperStructHoming.position = 2; // mm 235 mm

      if ((millis() / 5000) % 2)
      {
        stepperStructHoming.position = 10; // mm 252.5 mm
        Serial.println(100);
      }
      else
      {
        Serial.println(0);
      }
      loopStepper(stepperStructHoming);
    }
  }

  // Test Oled
  while (false)
  {
    uint16_t bat_power_voltage_int = analogRead(BAT_POWER_PIN);
    uint16_t bat_elec_voltage_int = analogRead(BAT_ELEC_PIN);
    float bat_power_voltage = bat_power_voltage_int * 6.935283019 * 3300.0f / 4096.0f; // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )
    float bat_elec_voltage = bat_elec_voltage_int * 7.3514 * 3300.0f / 4096.0f;        // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )

    Serial.print("Power: ");
    Serial.print(bat_power_voltage_int);
    Serial.print(", ");
    Serial.println(bat_power_voltage);
    Serial.print(", ");
    Serial.println(batt_to_str(bat_power_voltage));

    Serial.print("Elec: ");
    Serial.print(bat_elec_voltage_int);
    Serial.print(", ");
    Serial.println(bat_elec_voltage);
    Serial.print(", ");
    Serial.println(batt_to_str(bat_elec_voltage));

    // drawLCD(current_score);
    // current_score = (millis()/100)%256;
    loopOled(current_score, remaining_time_s, teamIsBlue, bat_elec_voltage, bat_power_voltage);
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
  io.myPinMode(BAT_POWER_PIN, INPUT);

  myPersistentServos[0] = new PersistentServo(SERVO_1);

  // setupVL53L0XDual(&io);
  setupVL53L0XSingle();

  setupAccelStepPami();

  setupMPU6050();

  setupOled(true, SUPERSTAR);

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
    float bat_12V_voltage = analogRead(BAT_POWER_PIN) * 6.23 * 3300 / 4096; // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )
    loopOled((millis() - timestamp_start_match) / 1000, teamIsBlue, bat_12V_voltage);
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

  float bat_12V_voltage = analogRead(BAT_POWER_PIN) * 6.23 * 3300 / 4096; // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )

  loopOled((millis() - timestamp_start_match) / 1000, teamIsBlue, bat_12V_voltage);

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

uint8_t digital_io_read;
uint16_t digital_io_output = 0;
uint8_t enables;
int8_t vacuum_1_enable_pump = 0;
int8_t vacuum_1_release = 0;
int8_t vacuum_2_enable_pump = 0;
int8_t vacuum_2_release = 0;
void loop()
{
  uint32_t timings[32];
  int l_time = 0;
  timings[l_time++] = millis();
  double pressure = 0; // readPressure();
  float bat_power_voltage = 0;
  float bat_elec_voltage = 0;
  for (int i = 0; i < 1; i++)
  {

    if (false)
    {
      if ((millis() / 1000) % 2)
      {
        dxl.ledOn(myAX12s[0].id);
        dxl.ledOn(myAX12s[1].id);
        dxl.ledOn(myAX12s[2].id);
        dxl.ledOn(255);
      }
      else
      {
        dxl.ledOff(myAX12s[0].id);
        dxl.ledOff(myAX12s[1].id);
        dxl.ledOff(myAX12s[2].id);
        dxl.ledOff(255);
      }
    }

    // updateDynamixelsInfo();

    bat_power_voltage = analogRead(BAT_POWER_PIN) * 6.935283019 * 3300.0f / 4096.0f; // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )
    bat_elec_voltage = analogRead(BAT_ELEC_PIN) * 7.3514 * 3300.0f / 4096.0f;        // ( (R1+R2)/R1 ) * ( max_ADC_Volt / max_ADC_value )

    // Digital inputs
    digital_io_read = 0;
    for (int pin_id = 0; pin_id < 4; pin_id++)
    {
      digital_io_read |= io.myDigitalRead(100 + pin_id) << pin_id;
    }

    // Transistors
    /*io.myDigitalWrite(TRANSISTOR_1_PIN, digital_io_output & (0x1 << 0));
    io.myDigitalWrite(TRANSISTOR_2_PIN, digital_io_output & (0x1 << 1));
    io.myDigitalWrite(TRANSISTOR_3_PIN, digital_io_output & (0x1 << 2));
    io.myDigitalWrite(TRANSISTOR_4_PIN, digital_io_output & (0x1 << 3));*/

    io.myDigitalWrite(TRANSISTOR_2_PIN, vacuum_1_release);
    io.myDigitalWrite(TRANSISTOR_4_PIN, vacuum_1_enable_pump);

    // io.myDigitalWrite(SERVO_POWER_ENABLE_PIN, enables & (0x1 << 0));

    for (int j = 0; j < 1; j++) // run loopCAN more often than slow I2C calls
    {
      // Serial.println("loopCan");
      loopCAN(current_score, &SERVO_1_msg, &SERVO_2_msg, &stepperStruct, &stepper_info, bat_power_voltage, bat_elec_voltage, digital_io_read, digital_io_output, enables, remaining_time_s, teamIsBlue,
              myAX12s[0].commands, myAX12s[1].commands, myAX12s[2].commands, myAX12s[3].commands, myAX12s[4].commands, myAX12s[5].commands,
              myAX12s[0].infos, myAX12s[1].infos, myAX12s[2].infos, myAX12s[3].infos, myAX12s[4].infos, myAX12s[5].infos,
              vacuum_1_enable_pump, vacuum_1_release, vacuum_2_enable_pump, vacuum_2_release);

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

      if (stepperStruct.mode != 0)
      {
        io.myDigitalWrite(STEPPER_POWER_ENABLE_PIN, HIGH);
      }
      else
      {
        io.myDigitalWrite(STEPPER_POWER_ENABLE_PIN, LOW);
      }

      for (int i_timing_actuators = 0; i_timing_actuators < 5; i_timing_actuators++)
      {
        for (int i_steppers = 0; i_steppers < 20; i_steppers++) // run loopStepper more often than slow stuff
        {
          stepper_info = loopStepper(stepperStruct);
        }

        updateServos(myPersistentServos);
        updateDynamixels();
      }
    }
    // delay(5);
  }

  loopOled(current_score, remaining_time_s, teamIsBlue, bat_elec_voltage, bat_power_voltage);

  showTeamColor(teamIsBlue);

  /*Serial.println("### Timings: ###");

  for (int i_time_print = 1; i_time_print < l_time; i_time_print++)
  {
    Serial.print(i_time_print);
    Serial.print(": ");
    Serial.print(timings[i_time_print] - timings[i_time_print - 1]);
    Serial.print(", ");
  }
  Serial.println("-------");*/

  // Serial.println(stepper_info.distance_to_go);
  // Serial.println(myPersistentServos[0]->angle);
  // Serial.println(current_score);
}
#endif
