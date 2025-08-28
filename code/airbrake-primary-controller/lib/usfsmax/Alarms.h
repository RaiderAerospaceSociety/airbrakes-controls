/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef Alarms_h
#define Alarms_h

#include "def.h"
#include "Types.h"

#define LEDPIN_PINMODE                pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
#define LEDPIN_ON                     digitalWrite(LED_PIN, LOW);
#define LEDPIN_OFF                    digitalWrite(LED_PIN, HIGH);
#define LEDPIN_TOGGLE                 if(digitalRead(LED_PIN)) digitalWrite(LED_PIN, LOW); else digitalWrite(LED_PIN, HIGH);

class Alarms
{
  public:
                                      Alarms();
     static void                      blink_blueLED(uint8_t num, uint8_t ontime,uint8_t repeat);
     static void                      toggle_blueLED();
     static void                      blueLEDon();
     static void                      blueLEDoff();
  private:
};

#endif // Alarms_h

