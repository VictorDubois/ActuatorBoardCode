#pragma once

#include <jm_PCF8574.h>                   // https://github.com/jmparatte/jm_PCF8574
#define LCM2004A_I2C_ADR1 ((uint8_t)0x3F) // default PCF8574A I2C address
#define LCM2004A_I2C_ADR2 ((uint8_t)0x27) // alternative PCF8574 I2C address

struct IOPotentiallyExpended
{
  jm_PCF8574 io_exp; // I2C address fixed later by begin(...)

  bool begin(uint8_t addr)
  {
    return io_exp.begin(addr);
  }

  bool begin()
  {
    return io_exp.begin(LCM2004A_I2C_ADR1);
  }

  int myDigitalRead(uint8_t pin)
  {
    if (pin >= 100)
    {
      return io_exp.digitalRead(pin - 100);
    }
    else
    {
      return digitalRead(pin);
    }
  }

  void myDigitalWrite(uint8_t PIN, uint8_t VALUE)
  {
    if (PIN >= 100)
    {
      io_exp.digitalWrite(PIN - 100, VALUE);
    }
    else
    {
      digitalWrite(PIN, VALUE);
    }
  }

  void myPinMode(uint8_t PIN, uint8_t VALUE)
  {
    if (PIN >= 100)
    {
      io_exp.pinMode(PIN - 100, VALUE);
    }
    else
    {
      pinMode(PIN, VALUE);
    }
  }
};