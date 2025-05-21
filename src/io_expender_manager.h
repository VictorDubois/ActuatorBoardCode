#pragma once

#include <jm_PCF8574.h> // https://github.com/jmparatte/jm_PCF8574
#define IO_EXP_I2C_ADDRESS 32

struct IOPotentiallyExpended
{
  jm_PCF8574 io_exp; // I2C address fixed later by begin(...)

  bool begin(uint8_t addr)
  {

    bool result = io_exp.begin(addr);
    if (result)
    {
      Serial.println("IO exp init SUCCESS!");
    }
    else
    {
      Serial.println("IO exp init failed");
    }

    return result;
  }

  bool begin()
  {
    return begin(IO_EXP_I2C_ADDRESS);
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