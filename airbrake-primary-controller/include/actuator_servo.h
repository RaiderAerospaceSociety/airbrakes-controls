#pragma once

#include <Arduino.h>

typedef struct {
  uint16_t min_us;
  uint16_t max_us;
  uint16_t cmd_us;
  bool open; // true if at/open target
} ServoStatus;

// Initialize PWM and optionally run boot sweep (blocking under setup mutex)
void servoInit();

// Start control or bench task depending on build flag
void servoStartTask();

// Immediate commands
void servoWriteUS(uint16_t us);
void servoOpen();
void servoClose();
void servoCenter();

// Status
ServoStatus servoGetStatus();

