/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#include "Arduino.h"
#include "I2Cdev.h"

I2Cdev::I2Cdev(TwoWire* wire)
{
  _I2C_Bus = wire;
}

uint8_t I2Cdev::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
  _I2C_Bus->beginTransmission(address);
  _I2C_Bus->write(subAddress);
  _I2C_Bus->endTransmission(false);
  _I2C_Bus->requestFrom(address, (uint8_t) 1);
  data = _I2C_Bus->read();
  return data;
}

void I2Cdev::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  _I2C_Bus->beginTransmission(address);
  _I2C_Bus->write(subAddress);
  _I2C_Bus->endTransmission(false);
  _I2C_Bus->requestFrom(address, count);
  uint8_t i = 0;
  while (_I2C_Bus->available())
  {
    dest[i++] = _I2C_Bus->read();
  }
}

void I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  _I2C_Bus->beginTransmission(devAddr);
  _I2C_Bus->write(regAddr);
  _I2C_Bus->write(data);
  _I2C_Bus->endTransmission();
}

void I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  _I2C_Bus->beginTransmission(devAddr);
  _I2C_Bus->write(regAddr);
  for (uint8_t i = 0; i < length; i++)
  {
    _I2C_Bus->write(data[i]);
  }
  _I2C_Bus->endTransmission();
}

void I2Cdev::I2Cscan()
{
  // Optional utility; not used by our task
}

