/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef Globals_h
#define Globals_h

#include "def.h"
#include "Types.h"
#include <FS.h>

// Intermediate data handling variables
float                                   sensor_point[3];
int16_t                                 gyroADC[2][3];
int16_t                                 accADC[2][3];
int16_t                                 magADC[2][3];
int32_t                                 baroADC[2];

// Timing variables
uint32_t                                Begin;
uint32_t                                Acq_time;
uint32_t                                Start_time   = 0;
float                                   TimeStamp    = 0.0f;
uint32_t                                currentTime  = 0;
uint32_t                                previousTime = 0;
uint8_t                                 serial_input = 0;
uint32_t                                last_refresh = 0;
uint32_t                                delt_t       = 0;
uint32_t                                cycleTime    = 0;

// Interrupt/state flags
volatile uint8_t                        data_ready[2]   = {1, 1};
volatile uint16_t                       calibratingG[2] = {0, 0};

// Calibration-related variables
full_adv_cal_t                          gyrocal[2];
full_adv_cal_t                          ellipsoid_magcal[2];
full_adv_cal_t                          accelcal[2];
full_adv_cal_t                          final_magcal[2];
uint8_t                                 GyroCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 EllipMagCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 AccelCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 FineMagCal_buff[sizeof(full_adv_cal_t)];
float                                   mag_calData[2][3];
float                                   dps_per_count = DPS_PER_COUNT;
float                                   g_per_count   = G_PER_COUNT;
float                                   UT_per_Count  = MMC5983MA_UT_PER_COUNT;
float                                   Mv_Cal        = 0.0f;
float                                   Mh_Cal        = 0.0f;
float                                   M_Cal         = 0.0f;
float                                   Del_Cal       = 0.0f;
uint8_t                                 cube_face     = 0;
uint8_t                                 face_rotation = 0;

// USFSMAX-related variables
CoProcessorConfig_t                     Cfg[2];
uint8_t                                 algostatus[2];
uint8_t                                 eventStatus[2];
int16_t                                 QT_Timestamp[2];
uint8_t                                 cfg_buff[sizeof(CoProcessorConfig_t)];
uint8_t                                 EulerQuatFlag        = OUTPUT_EULER_ANGLES;
uint8_t                                 ScaledSensorDataFlag = SCALED_SENSOR_DATA;
uint8_t                                 cal_status[2]        = {0, 0};
uint8_t                                 gyroCalActive[2]     = {0, 0};
uint8_t                                 Quat_flag[2]         = {0, 0};
uint8_t                                 Gyro_flag[2]         = {0, 0};
uint8_t                                 Acc_flag[2]          = {0, 0};
uint8_t                                 Mag_flag[2]          = {0, 0};
uint8_t                                 Baro_flag[2]         = {0, 0};
float                                   Rsq                  = 0.0f;

// IMU-related variables (subset we consume)
int16_t                                 accLIN[2][3];
int16_t                                 grav[2][3];
float                                   acc_LIN[2][3];
float                                   Mx[2], My[2];
float                                   gyroData[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   accData[2][3]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   magData[2][3]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   qt[2][4]       = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
float                                   QT[2][4]       = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
float                                   angle[2][2]    = {0.0f, 0.0f, 0.0f, 0.0f};
float                                   ANGLE[2][2]    = {0.0f, 0.0f, 0.0f, 0.0f};
float                                   heading[2]     = {0.0f, 0.0f};
float                                   HEADING[2]     = {0.0f, 0.0f};

#endif // Globals_h
