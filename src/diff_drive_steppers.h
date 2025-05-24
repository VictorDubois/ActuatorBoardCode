/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper motor.
 */

#include <TMCStepper.h> //https://github.com/teemuatlut/TMCStepper
#include <HardwareSerial.h>

#define END_STOP_PIN 36

#define EN_PIN_1 5            // Enable
#define DIR_PIN_1 12          // Direction
#define STEP_PIN_1 11         // Step
#define SW_RX_1 2             // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_1 38            // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT_1 Serial1 // TMC2208/TMC2224 HardwareSerial port

#define EN_PIN_2 5            // Enable
#define DIR_PIN_2 6           // Direction
#define STEP_PIN_2 8          // Step
#define SW_RX_2 2             // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_2 15            // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT_2 Serial2 // TMC2208/TMC2224 HardwareSerial port

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

enum trajectoryStep
{
  WAITING,
  TO_RAMP,
  ON_RAMP,
  AFTER_RAMP,
  TURN,
  BACKUP,
  TOWARD_EDGE,
  END
};
// HardwareSerial MySerial0(0);

TMC2208Stepper driver_left = TMC2208Stepper(&Serial1, R_SENSE);
TMC2208Stepper driver_right = TMC2208Stepper(&Serial2, R_SENSE);

#define ENTRAXE_MM 90.f
#define WHEEL_RADIUS_MM 30.0f
#define MICROSTEP_FACTOR 1 // 4?
#define STEPS_PER_REVOLUTION (360 / (1.8f * MICROSTEP_FACTOR))

#define DIST_MM_PER_REVOLUTION (WHEEL_RADIUS_MM * M_PI) // Distance traveled for a fulli wheel revolution (in mm)
#define STEPS_PER_MM_THEORY (STEPS_PER_REVOLUTION / DIST_MM_PER_REVOLUTION)
#define STEPS_PER_MM (STEPS_PER_MM_THEORY * 1.0f)
#define STEPS_PER_DEG_THEORY ((ENTRAXE_MM * STEPS_PER_REVOLUTION) / (DIST_MM_PER_REVOLUTION * 360.0f)) // Nb of step it takes to rotate 1 deg (old value: 4.7)
#define STEPS_PER_DEG (STEPS_PER_DEG_THEORY * 1.0f)

constexpr float steps_per_mm_pami = STEPS_PER_MM;
constexpr float diff_steps_per_deg_pami = STEPS_PER_DEG;
constexpr uint8_t to_mA_accel_pami = 50;
int32_t last_set_current_pami = 0;
int32_t last_set_position_pami = 0;
int32_t targetLeft = 0;
int32_t targetRight = 0;

#include <AccelStepper.h>
AccelStepper stepper_left = AccelStepper(stepper_left.DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper_right = AccelStepper(stepper_left.DRIVER, STEP_PIN_2, DIR_PIN_2);

trajectoryStep targetUpdatedTo = trajectoryStep::WAITING;
trajectoryStep currentStep = trajectoryStep::TO_RAMP;

void setupAccelStepPami()
{
  // MySerial0.begin(115200, SERIAL_8N1, SW_RX, SW_TX);
  Serial1.begin(115200, SERIAL_8N1, SW_RX_1, SW_TX_1);
  Serial2.begin(115200, SERIAL_8N1, SW_RX_2, SW_TX_2);

  driver_left.begin();            // Initiate pins and registeries
  driver_left.rms_current(12000); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  // driver_left.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver_left.pwm_autoscale(1);
  driver_left.microsteps(4);

  driver_right.begin();            // Initiate pins and registeries
  driver_right.rms_current(12000); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  // driver_right.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver_right.pwm_autoscale(1);
  driver_right.microsteps(4);

  stepper_left.setMaxSpeed(500 * steps_per_mm_pami);      // 100mm/s @ 80 steps/mm
  stepper_left.setAcceleration(1000 * steps_per_mm_pami); // 2000mm/s^2
  stepper_left.setEnablePin(EN_PIN_1);
  stepper_left.setPinsInverted(false, false, true);
  stepper_left.enableOutputs();

  stepper_right.setMaxSpeed(500 * steps_per_mm_pami);      // 100mm/s @ 80 steps/mm
  stepper_right.setAcceleration(1000 * steps_per_mm_pami); // 2000mm/s^2
  stepper_right.setEnablePin(EN_PIN_2);
  stepper_right.setPinsInverted(false, false, true);
  stepper_right.enableOutputs();

  pinMode(END_STOP_PIN, INPUT_PULLUP);
}

void loopStepperPami(bool stop, bool penche_gauche, bool penche_droite, bool penche_pente, bool teamIsBlue)
{
  bool stillWorkToDo = false;
  float speedLeft = 1500 * steps_per_mm_pami;
  float speedRight = 1500 * steps_per_mm_pami;

  if (stop)
  {
    Serial.println("stop");
    stepper_left.setSpeed(0);
    stepper_right.setSpeed(0);
    stepper_left.runSpeed();
    stepper_right.runSpeed();
    return;
  }

  switch (currentStep)
  {
  case trajectoryStep::TO_RAMP:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::TO_RAMP)
    {
      targetUpdatedTo = trajectoryStep::TO_RAMP;
      targetLeft += 550 * steps_per_mm_pami;  // 600 - demi longueur du PAMI
      targetRight += 550 * steps_per_mm_pami; // 600 - demi longueur du PAMI
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      Serial.println("Start ON_RAMP");
      currentStep = ON_RAMP;
    }
    break;
  case trajectoryStep::ON_RAMP:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::ON_RAMP)
    {
      targetUpdatedTo = trajectoryStep::ON_RAMP;
      targetLeft += 404 * steps_per_mm_pami;
      targetRight += 404 * steps_per_mm_pami;
    }

    speedLeft = 100 * steps_per_mm_pami;
    speedRight = 100 * steps_per_mm_pami;
    if (penche_gauche)
    {
      targetLeft += 10;
      targetRight -= 10;
      speedLeft /= 0.9;
      speedRight *= 0.9;
    }
    if (penche_droite)
    {
      targetLeft += 10;
      targetRight -= 10;
      speedLeft /= 0.9;
      speedRight *= 0.9;
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      // Todo: use accelerometer
      Serial.println("Start AFTER_RAMP");
      currentStep = AFTER_RAMP;
    }
    break;
  case trajectoryStep::AFTER_RAMP:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::AFTER_RAMP)
    {
      targetUpdatedTo = trajectoryStep::AFTER_RAMP;
      targetLeft += 200 * steps_per_mm_pami;
      targetRight += 200 * steps_per_mm_pami;
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      Serial.println("Start TURN");
      currentStep = trajectoryStep::TURN;
    }
    break;
  case trajectoryStep::TURN:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::TURN)
    {
      targetUpdatedTo = trajectoryStep::TURN;
      if (teamIsBlue)
      {
        targetLeft -= 90.0f * diff_steps_per_deg_pami;
        targetRight += 90.0f * diff_steps_per_deg_pami;
      }
      else
      {
        targetLeft += 90.0f * diff_steps_per_deg_pami;
        targetRight -= 90.0f * diff_steps_per_deg_pami;
      }
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      Serial.println("Start BACKUP");
      currentStep = trajectoryStep::BACKUP;
    }
    break;
  case trajectoryStep::BACKUP:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::BACKUP)
    {
      targetUpdatedTo = trajectoryStep::BACKUP;
      targetLeft -= 200 * steps_per_mm_pami;
      targetRight -= 200 * steps_per_mm_pami;
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      Serial.println("Start TOWARD_EDGE");
      currentStep = trajectoryStep::TOWARD_EDGE;
    }
    break;
  case trajectoryStep::TOWARD_EDGE:
    stepper_left.enableOutputs();
    stepper_right.enableOutputs();
    if (targetUpdatedTo != trajectoryStep::TOWARD_EDGE)
    {
      targetUpdatedTo = trajectoryStep::TOWARD_EDGE;
      targetLeft += 360 * steps_per_mm_pami;
      targetRight += 360 * steps_per_mm_pami;
    }

    stepper_left.setMaxSpeed(speedLeft);
    stepper_right.setMaxSpeed(speedRight);
    stepper_left.moveTo(targetLeft);
    stepper_right.moveTo(targetRight);
    stillWorkToDo |= stepper_left.run();
    stillWorkToDo |= stepper_right.run();

    if (!stillWorkToDo)
    {
      Serial.println("END!");
      currentStep = trajectoryStep::END;
    }
    break;
  default:
    stepper_left.disableOutputs();
    stepper_right.disableOutputs();
    break;
  }

  return;
}

void disableSteppers()
{
  stepper_left.disableOutputs();
  stepper_right.disableOutputs();
}
