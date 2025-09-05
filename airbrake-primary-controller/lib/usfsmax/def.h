/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef def_h
#define def_h

#include "config.h"

#if defined(USFS_MAX)
  #define ACC_ORIENTATION(X, Y, Z)              {accData[sensorNUM][EAST]   = +X;     accData[sensorNUM][NORTH] = +Y;     accData[sensorNUM][UP]   = +Z;}
  #define GYRO_ORIENTATION(X, Y, Z)             {gyroData[sensorNUM][PITCH] = +X;     gyroData[sensorNUM][ROLL] = +Y;     gyroData[sensorNUM][YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)              {magData[sensorNUM][0]      = +X;     magData[sensorNUM][1]     = -Y;     magData[sensorNUM][2]    = +Z;}
#endif

#if defined(LSM6DSM_GYRO_LPF_167)
  #define LSM6DSM_GYRO_DLPF_CFG               0x02
#elif defined(LSM6DSM_GYRO_LPF_223)
  #define LSM6DSM_GYRO_DLPF_CFG               0x01
#elif defined(LSM6DSM_GYRO_LPF_314)
  #define LSM6DSM_GYRO_DLPF_CFG               0x00
#elif defined(LSM6DSM_GYRO_LPF_655)
  #define LSM6DSM_GYRO_DLPF_CFG               0x03
#else
  #define LSM6DSM_GYRO_DLPF_CFG               0x00
#endif

#if defined(LSM6DSM_ACC_LPF_ODR_DIV2)
  #define LSM6DSM_ACC_DLPF_CFG                0x00
#elif defined(LSM6DSM_ACC_LPF_ODR_DIV4)
  #define LSM6DSM_ACC_DLPF_CFG                0x01
#elif defined(LSM6DSM_ACC_LPF_ODR_DIV9)
  #define LSM6DSM_ACC_DLPF_CFG                0x02
#elif defined(LSM6DSM_ACC_LPF_ODR_DIV50)
  #define LSM6DSM_ACC_DLPF_CFG                0x03
#elif defined(LSM6DSM_ACC_LPF_ODR_DIV100)
  #define LSM6DSM_ACC_DLPF_CFG                0x04
#elif defined(LSM6DSM_ACC_LPF_ODR_DIV400)
  #define LSM6DSM_ACC_DLPF_CFG                0x05
#else
  #define LSM6DSM_ACC_DLPF_CFG                0x02
#endif

#if defined(ACC_SCALE_2)
  #define ACC_SCALE 0x00
  #define G_PER_COUNT 0.0000610f
#elif defined(ACC_SCALE_4)
  #define ACC_SCALE 0x02
  #define G_PER_COUNT 0.0001220f
#elif defined(ACC_SCALE_8)
  #define ACC_SCALE 0.0002440f
  #define G_PER_COUNT 0.0002440f
#else
  #define ACC_SCALE 0x01
  #define G_PER_COUNT 0.0004880f
#endif

#if defined(GYRO_SCALE_125)
  #define GYRO_SCALE 0x02
  #define DPS_PER_COUNT 0.004375f
#elif defined(GYRO_SCALE_250)
  #define GYRO_SCALE 0x00
  #define DPS_PER_COUNT 0.00875f
#elif defined(GYRO_SCALE_500)
  #define GYRO_SCALE 0x04
  #define DPS_PER_COUNT 0.0175f
#elif defined(GYRO_SCALE_1000)
  #define GYRO_SCALE 0x08
  #define DPS_PER_COUNT 0.035f
#else
  #define GYRO_SCALE 0x0C
  #define DPS_PER_COUNT 0.070f
#endif

#define MMC5983MA_UT_PER_COUNT 0.006103515625f
#define RPS_PER_DPS            0.01745329

#ifdef KELSEYVILLE_CA_USA
  #define M_V 42.9631f
  #define M_H 22.7568f
  #define MAG_DECLINIATION 13.7433f
#else
  #define M_V 42.9631f
  #define M_H 22.7568f
  #define MAG_DECLINIATION 13.7433f
#endif

#ifndef USFS_MAX
  #define USFS_MAX
#endif

#endif // def_h

