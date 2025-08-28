/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef config_h
#define config_h

#define MAX32660_SLV_ADDR   (0x57)

// Pins below are unused in our integration, but kept for compatibility
#define INT_PIN             33    // DRDY pin (your request)
#define LED_PIN             2     // Use a harmless GPIO; do not clash with BMP CS

// I2C instance and clock
#define SENSOR_0_WIRE_INSTANCE Wire
#define I2C_CLOCK            400000

// Output options
#define ENABLE_DHI_CORRECTOR 0x00
#define USE_2D_DHI_CORRECTOR 0x00

// Update period / calibration points (not used directly here)
#define UPDATE_PERIOD        100
#define CAL_POINTS           2048

// Euler angles output, scaled sensor data flags
#define SERIAL_DEBUG                     // Enable verbose init prints inside library
#define MAHONY_9DOF

// Example scales and filters
#define ACC_SCALE_16
#define GYRO_SCALE_2000
#define LSM6DSM_GYRO_LPF_167
#define LSM6DSM_ACC_LPF_ODR_DIV9
#define MMC5983MA_MAG_LPF  0x00
#define MMC5983MA_MAG_HPF  0x00
#define LPS22HB_BARO_LPF   0x0C
#define LPS22HB_BARO_HPF   0x00

// Fixed scales for MMC5983MA and LPS22HB (not adjustable in coprocessor)
#define MAG_SCALE           0x00
#define BARO_SCALE          0x00

// Output data rates (ODR) — minimal defaults (tuned in coprocessor)
#define ACC_ODR             0x00
#define GYRO_ODR            0x00
#define MAG_ODR             0x00
#define BARO_ODR            0x00

// High-pass filter config placeholders
#define LSM6DSM_ACC_DHPF_CFG  0x00
#define LSM6DSM_GYRO_DHPF_CFG 0x00

// AUX channels (unused; keep at 0)
#define AUX1_ODR            0x00
#define AUX2_ODR            0x00
#define AUX3_ODR            0x00
#define AUX1_SCALE          0x00
#define AUX2_SCALE          0x00
#define AUX3_SCALE          0x00
#define AUX1_LPF            0x00
#define AUX1_HPF            0x00
#define AUX2_LPF            0x00
#define AUX2_HPF            0x00
#define AUX3_LPF            0x00
#define AUX3_HPF            0x00

// Output selection
#define OUTPUT_EULER_ANGLES 1
#define SCALED_SENSOR_DATA  1

// Quaternion downsample divider
#define QUAT_DIV            0x05

// Location for magnetic constants
#define KELSEYVILLE_CA_USA

#endif
