/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper motor.
 */

#include <TMCStepper.h>
#include <HardwareSerial.h>

#define END_STOP_PIN           55

#define EN_PIN           14 // Enable
#define DIR_PIN          13 // Direction
#define STEP_PIN         2 // Step
#define SW_RX            15 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            16 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port

#define R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075


HardwareSerial MySerial0(0);
TMC2208Stepper driver = TMC2208Stepper(&MySerial0, R_SENSE);

constexpr uint32_t steps_per_mm = 80;
constexpr uint8_t to_mA_accel = 50;
StepperInfo stepper_info;

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setupAccelStep() {
    MySerial0.begin(9600, SERIAL_8N1, SW_RX, SW_TX);

    

    driver.begin();             // Initiate pins and registeries
    driver.rms_current(6000);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driver.en_pwm_mode(1);      // Enable extremely quiet stepping
    driver.pwm_autoscale(1);
    driver.microsteps(16);

    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

    stepper_info.homing_sequences_done = 0;

    pinMode(END_STOP_PIN, INPUT_PULLUP);
}

StepperInfo loopStepper(Stepper& stepperStruct) {

    if(driver.rms_current() != stepperStruct.current * to_mA_accel)
    {
        // Check, to avoid calling unecessarily
        driver.rms_current(stepperStruct.current * to_mA_accel);
    }

    switch(stepperStruct.mode)
    {
      case stepper_mode::DISABLE:
        stepper.disableOutputs();
        break;
      case stepper_mode::POSITION:
        stepper.enableOutputs();
        stepper.moveTo(stepperStruct.position);

        if(stepper.acceleration() != stepperStruct.accel * steps_per_mm)
        {
            // Check, to avoid calling unecessarily
            stepper.setAcceleration(stepperStruct.accel * steps_per_mm);
        }

        if(stepper.maxSpeed() != stepperStruct.speed * steps_per_mm)
        {
            // Check, to avoid calling unecessarily
            stepper.setMaxSpeed(stepperStruct.speed * steps_per_mm);
        }

        stepper.run();
        stepper_info.distance_to_go = stepper.distanceToGo();
        break;
      case stepper_mode::SPEED:
        stepper.enableOutputs();
        stepper.setSpeed(stepperStruct.speed*steps_per_mm);
        stepper.runSpeed();
        break;
      case stepper_mode::HOMING:
        stepper.enableOutputs();
        stepper.setSpeed(-stepperStruct.speed*steps_per_mm);
        while(digitalRead(END_STOP_PIN) != LOW)
        {
            stepper.runSpeed();
        }
        stepper.setCurrentPosition(0);
        stepper_info.homing_sequences_done++;
        stepper.disableOutputs();
        break;
      default:
        stepper.disableOutputs();
        Serial.println("Unknown stepper mode!");
        break;
    }
    return stepper_info;
}
