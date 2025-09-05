/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#include "Arduino.h"
#include "Alarms.h"

Alarms::Alarms()
{
}

void Alarms::blink_blueLED(uint8_t num, uint8_t ontime,uint8_t repeat) 
{
  uint8_t i, r;
  for(r=0; r<repeat; r++)
  {
    for(i=0; i<num; i++)
    {
      Alarms::toggle_blueLED();
      delay(ontime);
    }
    delay(60);
  }
}

void Alarms::blueLEDon()
{
  LEDPIN_ON;
}

void Alarms::blueLEDoff()
{
  LEDPIN_OFF;
}

void Alarms::toggle_blueLED()
{
  LEDPIN_TOGGLE;
}

