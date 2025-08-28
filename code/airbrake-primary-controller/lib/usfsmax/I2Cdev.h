/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include "def.h"
#include <Wire.h>

class I2Cdev
{
    public:
                                        I2Cdev(TwoWire*);
         uint8_t                        readByte(uint8_t address, uint8_t subAddress);
         void                           readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
         void                           writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
         void                           writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
         void                           I2Cscan();
    private:
         TwoWire*                       _I2C_Bus;
};

#endif //_I2CDEV_H_

