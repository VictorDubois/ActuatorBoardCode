#include <cstring> // For memcpy
#include "CanStruct/can_structs.h"

using namespace CAN;

// Define a struct that matches your CAN message format
struct CanMessage
{
  uint16_t id;     // CAN message ID
  uint8_t dlc;     // Data Length Code (how many bytes of data)
  uint8_t data[8]; // Up to 8 bytes of data payload
};

//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#include <ACAN_ESP32.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL; // 1 Mb/s
bool pinged = false;

void encodeAX12Read(CANMessage *frame, const AX12Read a_ax12_read)
{
  frame->rtr = 0;
  frame->len = sizeof(AX12Read);
  frame->data[0] = a_ax12_read.hardwareErrorStatus & 0xFF;
  frame->data[1] = (a_ax12_read.current_position >> 8) & 0xFF;
  frame->data[2] = a_ax12_read.current_position & 0xFF;
  frame->data[3] = a_ax12_read.presentTemperature & 0xFF;
  frame->data[4] = (a_ax12_read.presentCurrent >> 8) & 0xFF;
  frame->data[5] = a_ax12_read.presentCurrent & 0xFF;
  frame->data[6] = a_ax12_read.moving & 0xFF;
  frame->data[7] = a_ax12_read.mode & 0xFF;
}

void decodeAX12Write(const CANMessage *frame, AX12Write &a_ax12_write)
{
  a_ax12_write.mode = (uint8_t)frame->data[0];
  a_ax12_write.position = (((uint8_t)frame->data[1]) << 8) | ((uint8_t)frame->data[2]);
  a_ax12_write.max_accel = (uint8_t)frame->data[3];
  a_ax12_write.max_speed = (uint8_t)frame->data[4];
  a_ax12_write.torque_enable = (uint8_t)frame->data[5];
  a_ax12_write.temperatureLimit = (uint8_t)frame->data[6];
  a_ax12_write.currentLimit = (uint8_t)frame->data[7];
}

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------
void setupCAN()
{
  //--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;

  settings.mRxPin = GPIO_NUM_48; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_47; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode != 0)
  {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }

  CANMessage frame;
  frame.id = 0;
  const bool ok = ACAN_ESP32::can.tryToSend(frame);
}

//----------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------
void loopCAN(uint8_t &a_current_score, ServoMessage *SERVO_1_msg, ServoMessage *SERVO_2_msg, Stepper *stepperStruct, const StepperInfo *stepper_info, const uint16_t bat_power_voltage, const uint16_t bat_elec_voltage, uint8_t digital_io_read, uint16_t &a_digital_io_output, uint8_t &a_enables, int8_t &a_remaining_time_s, int8_t &a_is_blue,
             AX12Write &ax12_w1, AX12Write &ax12_w2, AX12Write &ax12_w3, AX12Write &ax12_w4, AX12Write &ax12_w5, AX12Write &ax12_w6, const AX12Read &ax12_r1, const AX12Read &ax12_r2, const AX12Read &ax12_r3, const AX12Read &ax12_r4, const AX12Read &ax12_r5, const AX12Read &ax12_r6,
             int8_t &vacuum_1_enable_pump, int8_t &vacuum_1_release, int8_t &vacuum_2_enable_pump, int8_t &vacuum_2_release)
{
  // Serial.print("."); // debug
  pinged = true; // force sending a CAN message
  CANMessage frame;
  if (gBlinkLedDate < millis())
  {
    gBlinkLedDate += 100;
    // digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;

    bool l_print_debug = false;
    if (l_print_debug)
    {
      Serial.print("Sent: ");
      Serial.print(gSentFrameCount);
      Serial.print(" ");
      Serial.print("Receive: ");
      Serial.print(gReceivedFrameCount);
      Serial.println(" ");
      /*Serial.print (" STATUS 0x") ;
      Serial.print (TWAI_STATUS_REG, HEX) ;
      Serial.print (" RXERR ") ;
      Serial.print (TWAI_RX_ERR_CNT_REG) ;
      Serial.print (" TXERR ") ;
      Serial.println (TWAI_TX_ERR_CNT_REG) ;*/
    }
    if (pinged)
    {
      frame.id = 42;
      frame.rtr = 0;
      frame.len = 1;
      frame.data[0] = 12;
      const bool ok = ACAN_ESP32::can.tryToSend(frame);
      if (ok)
      {
        gSentFrameCount += 1;
      }
    }

    frame.id = can_ids::ANALOG_SENSORS;
    frame.rtr = 0;
    frame.len = sizeof(AnalogSensors);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    uint16_t batt_mv = bat_power_voltage;
    frame.data[1] = batt_mv & 0xFF;
    frame.data[0] = batt_mv >> 8;
    batt_mv = bat_elec_voltage;
    frame.data[3] = batt_mv & 0xFF;
    frame.data[2] = batt_mv >> 8;
    ACAN_ESP32::can.tryToSend(frame);

    frame.id = can_ids::STEPPER_INFO;
    frame.rtr = 0;
    frame.len = sizeof(StepperInfo);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    frame.data[2] = stepper_info->homing_sequences_done;
    frame.data[1] = stepper_info->distance_to_go & 0xFF;
    frame.data[0] = stepper_info->distance_to_go >> 8;
    ACAN_ESP32::can.tryToSend(frame);

    frame.id = can_ids::DIGITAL_INPUTS;
    frame.rtr = 0;
    frame.len = sizeof(DigitalInputs);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    frame.data[0] = digital_io_read;
    ACAN_ESP32::can.tryToSend(frame);

    /*

    frame.id = can_ids::AX12_R1;
    encodeAX12Read(&frame, ax12_r1);
    ACAN_ESP32::can.tryToSend(frame);

    frame.id = can_ids::AX12_R2;
    encodeAX12Read(&frame, ax12_r2);
    ACAN_ESP32::can.tryToSend(frame);

    frame.id = can_ids::AX12_R3;
    encodeAX12Read(&frame, ax12_r3);
    ACAN_ESP32::can.tryToSend(frame);

    frame.id = can_ids::AX12_R4;
    encodeAX12Read(&frame, ax12_r4);
    ACAN_ESP32::can.tryToSend(frame);*/

    /*frame.id = can_ids::AX12_R5;
    encodeAX12Read(&frame, ax12_r5);
    ACAN_ESP32::can.tryToSend (frame);

    frame.id = can_ids::AX12_R6;
    encodeAX12Read(&frame, ax12_r6);
    ACAN_ESP32::can.tryToSend (frame);*/
  }

  while (ACAN_ESP32::can.receive(frame))
  {
    gReceivedFrameCount += 1;
    if (frame.id == SERVO_1)
    {
      SERVO_1_msg->angle_s1 = frame.data[0];
      SERVO_1_msg->speed_s1 = frame.data[1]; // 0 means disable the servo
      SERVO_1_msg->angle_s2 = frame.data[2];
      SERVO_1_msg->speed_s2 = frame.data[3]; // 0 means disable the servo
      SERVO_1_msg->angle_s3 = frame.data[4];
      SERVO_1_msg->speed_s3 = frame.data[5]; // 0 means disable the servo
      SERVO_1_msg->angle_s4 = frame.data[6];
      SERVO_1_msg->speed_s4 = frame.data[7]; // 0 means disable the servo

      pinged = true;
    }
    else if (frame.id == SERVO_2)
    {
      SERVO_2_msg->angle_s1 = frame.data[0];
      SERVO_2_msg->speed_s1 = frame.data[1]; // 0 means disable the servo
      SERVO_2_msg->angle_s2 = frame.data[2];
      SERVO_2_msg->speed_s2 = frame.data[3]; // 0 means disable the servo
      SERVO_2_msg->angle_s3 = frame.data[4];
      SERVO_2_msg->speed_s3 = frame.data[5]; // 0 means disable the servo
      SERVO_2_msg->angle_s4 = frame.data[6];
      SERVO_2_msg->speed_s4 = frame.data[7]; // 0 means disable the servo

      pinged = true;
    }
    else if (frame.id == STEPPER_CMD)
    {
      stepperStruct->speed = (frame.data[0] << 8) | frame.data[1];
      stepperStruct->accel = (frame.data[2] << 8) | frame.data[3];
      stepperStruct->position = (frame.data[4] << 8) | frame.data[5];
      stepperStruct->current = frame.data[6];
      stepperStruct->mode = frame.data[7];
    }
    else if (frame.id == can_ids::SCORE)
    {
      a_current_score = frame.data[0];
      a_remaining_time_s = frame.data[1];
      a_is_blue = frame.data[2];
    }
    else if (frame.id == can_ids::DIGITAL_OUTPUTS)
    {
      vacuum_1_enable_pump = frame.data[0];
      vacuum_1_release = frame.data[1];
      vacuum_2_enable_pump = frame.data[2];
      vacuum_2_release = frame.data[3];
      /*
      // old syntax, we do not respect the official format anymore, it is late and I want to sleep
      a_digital_io_output = (frame.data[0] << 8) | frame.data[1];
      a_enables = frame.data[2];*/
    }
    else if (frame.id == can_ids::AX12_W1)
    {
      decodeAX12Write(&frame, ax12_w1);
    }
    else if (frame.id == can_ids::AX12_W2)
    {
      decodeAX12Write(&frame, ax12_w2);
    }
    else if (frame.id == can_ids::AX12_W3)
    {
      decodeAX12Write(&frame, ax12_w3);
    }
    else if (frame.id == can_ids::AX12_W4)
    {
      decodeAX12Write(&frame, ax12_w4);
    }
    else if (frame.id == can_ids::AX12_W5)
    {
      decodeAX12Write(&frame, ax12_w5);
    }
    else if (frame.id == can_ids::AX12_W6)
    {
      decodeAX12Write(&frame, ax12_w6);
    }
  }
}

//----------------------------------------------------------------------------------------
