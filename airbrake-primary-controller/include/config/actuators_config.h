// Actuators configuration
#pragma once

// Enable/disable servo actuator
#ifndef SERVO_ENABLE
#define SERVO_ENABLE 1
#endif

// Bench mode: build a standalone servo test (disables FSM actuation)
#ifndef SERVO_BENCH
#define SERVO_BENCH 0
#endif

// Feather S3: use GPIO 12 for PWM output
#ifndef SERVO_PWM_PIN
#define SERVO_PWM_PIN 11
#endif

// Servo endpoints in microseconds (higher = extend)
#ifndef SERVO_MIN_US
#define SERVO_MIN_US 1000
#endif
#ifndef SERVO_MAX_US
#define SERVO_MAX_US 1400
#endif

// LEDC configuration (ESP32 PWM)
#ifndef SERVO_PWM_FREQ_HZ
#define SERVO_PWM_FREQ_HZ 50
#endif
#ifndef SERVO_PWM_RES_BITS
#define SERVO_PWM_RES_BITS 16
#endif
#ifndef SERVO_PWM_CHANNEL
#define SERVO_PWM_CHANNEL 4
#endif

// Control task period (ms)
#ifndef SERVO_TASK_PERIOD_MS
#define SERVO_TASK_PERIOD_MS 20
#endif
