/****************************************************************************
 * Author : Victor Dubois
 ****************************************************************************/

#include <Wire.h>
#include "CanStruct/can_structs.h"
using namespace CAN;

#include "can_manager.h"
#include "stepper_manager.h"
#include "AX12_manager.h"

#include "constants.h"

#include "billig.h"
#include "devices_checks.h"

ServoMessage SERVO_1_msg;
ServoMessage SERVO_2_msg;
Stepper stepperStruct;

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

void setup()
{
  // For the PCB with the broken DCDC
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN_U3, OUTPUT);

  /*while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  // delay(1000);

  // Serial.setTimeout(100);
  Serial.println("Start Actuator board");
  Serial.flush();

  // Test dynamixel
  testDynamixels3(false);

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

  SERVO_1_msg.angle_s1 = 30;
  SERVO_1_msg.angle_s2 = 0;
  SERVO_1_msg.angle_s3 = 0;
  SERVO_1_msg.angle_s4 = 0;
  SERVO_1_msg.speed_s1 = 100;
  SERVO_1_msg.speed_s2 = 100;
  SERVO_1_msg.speed_s3 = 100;
  SERVO_1_msg.speed_s4 = 100;

  SERVO_2_msg.angle_s1 = 175;
  SERVO_2_msg.speed_s1 = 100;

  Serial.println("io pinmode done");

  testTransistors(false, io);

  setupOled();

  testDynamixels(false);

  int servoIndex = 0;
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_1, SERVO_1_msg.angle_s1, 100, 0.1, 25, 145);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_2, SERVO_1_msg.angle_s1, 100, 0.1, 0, 180);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_3, SERVO_1_msg.angle_s1, 100, 0.1, 0, 180);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_4, SERVO_1_msg.angle_s1, 100, 0.1, 30, 150);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_5, 175, 1, 1, 100, 175);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_6, 100, 255, 0, 112, 175);
  myPersistentServos[servoIndex++] = new PersistentServo(SERVO_7, 100, 255, 1, 110, 175);
  // myPersistentServos[servoIndex++] = new PersistentServo(SERVO_8); // Clicou instead
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);

  testServos(false, myPersistentServos);

  delay(15);
  Serial.println("Init AccelStep");
  setupAccelStep(io);

  // digitalWrite(SUCTION_CUP_PIN, HIGH);
  // digitalWrite(VALVE_PIN, HIGH);

  current_score = 0;

  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  turnlightsoff();

  setupCAN();
  Serial.println("Setup CAN done");
  Serial.flush();

  setupDynamixel();

  testDynamixels2(false);

  Serial.println("end of Setup");
  Serial.flush();

  // Test IOs
  testIOs(false, io);

  // Homing
  Serial.println("Start homing");

  Stepper stepperStructHoming;
  stepperStructHoming.current = 200; //*50mA
  stepperStructHoming.accel = 100;   // mm/s2
  stepperStructHoming.speed = 30;    // mm/s
  stepperStructHoming.position = 0;  // mm
  stepperStructHoming.mode = stepper_mode::HOMING;
  // stepperStructHoming.mode = stepper_mode::POSITION;
  loopStepper(stepperStructHoming, io);

  Serial.println("Homing done");
  Serial.flush();

  // Do not move
  stepperStruct.mode = stepper_mode::POSITION;
  stepperStruct.position = 0;

  initBillig(myPersistentServos);

  testPumps(false, io);

  testStepper(false, stepperStruct, io);

  testOled(false, current_score, remaining_time_s, teamIsBlue);
}

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
    blinkDynamixels(false);

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
    io.myDigitalWrite(TRANSISTOR_1_PIN, vacuum_2_release);
    io.myDigitalWrite(TRANSISTOR_3_PIN, vacuum_2_enable_pump);

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
          stepper_info = loopStepper(stepperStruct, io);
        }

        updateServos(myPersistentServos);
        updateDynamixels();
      }
      updateDynamixelsInfo();
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
