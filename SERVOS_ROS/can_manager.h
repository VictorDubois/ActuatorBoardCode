#include <cstring>  // For memcpy
#include "can_structs.h"

using namespace CAN;


// Define a struct that matches your CAN message format
struct CanMessage { 
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
#define LED_BUILTIN 2
#include <ACAN_ESP32.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 500UL * 1000UL ; // 500 kb/s
bool pinged = false;
//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------
void setupCAN () {
//--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE) ;
  //settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
  
  settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
  if (errorCode != 0) {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  CANMessage frame ;
  frame.id = 0;
  const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
}

//----------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loopCAN (uint8_t& a_current_score, ServoMessage* SERVO_1_msg, ServoMessage* SERVO_2_msg, Stepper* stepperStruct, StepperInfo* stepper_info, uint16_t bat_12V_voltage, uint8_t digital_io_read, uint16_t& a_digital_io_output, uint8_t& a_enables) {
  //Serial.print("."); // debug
  pinged = true; // force sending a CAN message
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 100 ;
    //digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;

    bool l_print_debug = true;
    if (l_print_debug)
    {
      Serial.print ("Sent: ") ;
      Serial.print (gSentFrameCount) ;
      Serial.print (" ") ;
      Serial.print ("Receive: ") ;
      Serial.print (gReceivedFrameCount) ;
      Serial.print (" ") ;
      /*Serial.print (" STATUS 0x") ;
      Serial.print (TWAI_STATUS_REG, HEX) ;
      Serial.print (" RXERR ") ;
      Serial.print (TWAI_RX_ERR_CNT_REG) ;
      Serial.print (" TXERR ") ;
      Serial.println (TWAI_TX_ERR_CNT_REG) ;*/
    }
    if (pinged){
      frame.id = 42;
      frame.rtr = 0;
      frame.len = 1;
      frame.data[0] = 12;
      const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
      if (ok) {
        gSentFrameCount += 1 ;
      }
    }

    frame.id = can_ids::ANALOG_SENSORS;
    frame.rtr = 0;
    frame.len = sizeof(AnalogSensors);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    uint16_t batt_mv = bat_12V_voltage;
    frame.data[1] = batt_mv%256;
    frame.data[0] = batt_mv>>8;
    ACAN_ESP32::can.tryToSend (frame) ;


    frame.id = can_ids::STEPPER_INFO;
    frame.rtr = 0;
    frame.len = sizeof(StepperInfo);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    frame.data[2] = stepper_info->homing_sequences_done;
    frame.data[1] = stepper_info->distance_to_go%256;
    frame.data[0] = stepper_info->distance_to_go>>8;
    ACAN_ESP32::can.tryToSend (frame) ;

    frame.id = can_ids::DIGITAL_INPUTS;
    frame.rtr = 0;
    frame.len = sizeof(DigitalInputs);
    for (int i = 0; i < 8; i++)
    {
      frame.data[i] = 0;
    }
    frame.data[0] = digital_io_read;
    ACAN_ESP32::can.tryToSend (frame) ;
  }

  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCount += 1 ;
    if(frame.id == SERVO_1)
    {
        SERVO_1_msg->angle_s1 = frame.data_s8[0];
        SERVO_1_msg->speed_s1 = frame.data_s8[1]; // 0 means disable the servo
        SERVO_1_msg->angle_s2 = frame.data_s8[2];
        SERVO_1_msg->speed_s2 = frame.data_s8[3]; // 0 means disable the servo
        SERVO_1_msg->angle_s3 = frame.data_s8[4];
        SERVO_1_msg->speed_s3 = frame.data_s8[5]; // 0 means disable the servo
        SERVO_1_msg->angle_s4 = frame.data_s8[6];
        SERVO_1_msg->speed_s4 = frame.data_s8[7]; // 0 means disable the servo

        pinged = true;
    }
    else if(frame.id == SERVO_2)
    {
        SERVO_2_msg->angle_s1 = frame.data_s8[0];
        SERVO_2_msg->speed_s1 = frame.data_s8[1]; // 0 means disable the servo
        SERVO_2_msg->angle_s2 = frame.data_s8[2];
        SERVO_2_msg->speed_s2 = frame.data_s8[3]; // 0 means disable the servo
        SERVO_2_msg->angle_s3 = frame.data_s8[4];
        SERVO_2_msg->speed_s3 = frame.data_s8[5]; // 0 means disable the servo
        SERVO_2_msg->angle_s4 = frame.data_s8[6];
        SERVO_2_msg->speed_s4 = frame.data_s8[7]; // 0 means disable the servo

        pinged = true;
    }
    else if(frame.id == STEPPER_CMD)
    {
        stepperStruct->speed    = frame.data_s8[0] << 8 | frame.data_s8[1];
        stepperStruct->accel    = frame.data_s8[2] << 8 | frame.data_s8[3];
        stepperStruct->position = frame.data_s8[4] << 8 | frame.data_s8[5];
        stepperStruct->current  = frame.data_s8[6];
        stepperStruct->mode     = frame.data_s8[7];
    }
    else if(frame.id == can_ids::SCORE)
    {
        a_current_score = frame.data[0];
    }
    else if(frame.id == can_ids::DIGITAL_OUTPUTS)
    {
        a_digital_io_output = frame.data_s8[0] << 8 | frame.data_s8[1];
        a_enables = frame.data_s8[2];
    }
  }

}
//----------------------------------------------------------------------------------------
