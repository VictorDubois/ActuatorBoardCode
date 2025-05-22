/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper motor.
 */
#pragma once
#include <TMCStepper.h> // https://github.com/adafruit/Adafruit_MPU6050
#include <HardwareSerial.h>
#include "CanStruct/can_structs.h"

#define END_STOP_PIN 43 // =Servo8

#define EN_PIN 5            // Enable
#define DIR_PIN 12          // Direction
#define STEP_PIN 11         // Step
#define SW_RX 2             // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX 38            // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// HardwareSerial MySerial0(0);

TMC2208Stepper driver = TMC2208Stepper(&Serial2, R_SENSE);

constexpr uint32_t steps_per_mm = 41;
constexpr uint8_t to_mA_accel = 50;
StepperInfo stepper_info;
int32_t last_set_current = 0;
int32_t last_set_position = 0;
auto homingTimeout = millis() + 15000;

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setupAccelStep()
{
  // MySerial0.begin(115200, SERIAL_8N1, SW_RX, SW_TX);
  Serial2.begin(115200, SERIAL_8N1, SW_RX, SW_TX);

  driver.begin();            // Initiate pins and registeries
  driver.rms_current(12000); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  // driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.microsteps(4);

  stepper.setMaxSpeed(500 * steps_per_mm);      // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(1000 * steps_per_mm); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  stepper_info.homing_sequences_done = 0;

  pinMode(END_STOP_PIN, INPUT_PULLUP);
}

StepperInfo loopStepper(Stepper &stepperStruct)
{

  // if(driver.rms_current() != stepperStruct.current * to_mA_accel)
  {
    /*Serial.println("Set rms_current to :");
    Serial.println(stepperStruct.current * to_mA_accel);
    Serial.println(driver.rms_current());
    // Check, to avoid calling unecessarily
    driver.rms_current(stepperStruct.current * to_mA_accel);
    Serial.println(driver.rms_current());*/
  }
  bool stillWorkToDo = true;
  switch (stepperStruct.mode)
  {
  case stepper_mode::DISABLE:
    last_set_position = 0;
    stepper.disableOutputs();
    break;
  case stepper_mode::POSITION:
    stepper.enableOutputs();

    /*if (stepper.acceleration() != stepperStruct.accel * steps_per_mm)
    {
      Serial.println("Set accel to :");
      Serial.println(stepperStruct.accel * steps_per_mm);
      Serial.println(stepper.acceleration());

      // Check, to avoid calling unecessarily
      stepper.setAcceleration(stepperStruct.accel * steps_per_mm);
      Serial.println(stepper.acceleration());
    }*/

    if (stepper.maxSpeed() != stepperStruct.speed * steps_per_mm)
    {
      Serial.println("Set MaxSpeed to :");
      Serial.println(stepperStruct.speed * steps_per_mm);
      Serial.println(stepper.maxSpeed());
      // Check, to avoid calling unecessarily
      stepper.setMaxSpeed(stepperStruct.speed * steps_per_mm);
      Serial.println(stepper.maxSpeed());
    }
    if (last_set_position != stepperStruct.position * steps_per_mm)
    {
      Serial.print("New Target : ");
      Serial.println(stepperStruct.position * steps_per_mm);
      stepper.moveTo(stepperStruct.position * steps_per_mm);
      last_set_position = stepperStruct.position * steps_per_mm;
    }
    for (int j = 0; j < 100; j++)
    {
      stillWorkToDo = stepper.run();
    }
    if (!stillWorkToDo)
    {
      // Serial.println("target reached!");
    }
    stepper_info.distance_to_go = stepper.distanceToGo() / steps_per_mm;
    break;
  case stepper_mode::SPEED:
    last_set_position = 0;
    stepper.enableOutputs();
    stepper.setSpeed(stepperStruct.speed * steps_per_mm);

    stepper.move(stepperStruct.position * steps_per_mm);
    stepper.runSpeed();
    break;
  case stepper_mode::HOMING:
    last_set_position = 0;
    stepper.enableOutputs();
    stepper.setSpeed(stepperStruct.speed * steps_per_mm);
    stepper.move(-1000 * steps_per_mm);
    homingTimeout = millis() + 15000;
    Serial.println("Start homing stepper");

    while (digitalRead(END_STOP_PIN) == HIGH)
    {

      Serial.println("END_STOP_PIN");
      Serial.println(digitalRead(END_STOP_PIN));
      if (millis() > homingTimeout)
      {
        Serial.println("Failed to home stepper: timeout reached");
        break;
      }
      stepper.runSpeed();
    }
    stepper.setCurrentPosition(0);
    stepper_info.homing_sequences_done++;
    stepper.disableOutputs();
    break;
  default:
    last_set_position = 0;
    stepper.disableOutputs();
    Serial.println("Unknown stepper mode!");
    break;
  }

  return stepper_info;
}
