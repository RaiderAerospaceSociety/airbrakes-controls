/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 */

#ifndef types_h
#define types_h

enum axes { EAST = 0, NORTH, UP };
enum attitudes { PITCH = 0, ROLL, YAW };

typedef struct {
  uint16_t cal_points;
  uint8_t  Ascale; uint8_t  AODR; uint8_t  Alpf; uint8_t  Ahpf;
  uint8_t  Gscale; uint8_t  GODR; uint8_t  Glpf; uint8_t  Ghpf;
  uint8_t  Mscale; uint8_t  MODR; uint8_t  Mlpf; uint8_t  Mhpf;
  uint8_t  Pscale; uint8_t  PODR; uint8_t  Plpf; uint8_t  Phpf;
  uint8_t  AUX1scale; uint8_t  AUX1ODR; uint8_t  AUX1lpf; uint8_t  AUX1hpf;
  uint8_t  AUX2scale; uint8_t  AUX2ODR; uint8_t  AUX2lpf; uint8_t  AUX2hpf;
  uint8_t  AUX3scale; uint8_t  AUX3ODR; uint8_t  AUX3lpf; uint8_t  AUX3hpf;
  float    m_v; float m_h; float m_dec; uint8_t quat_div;
} CoProcessorConfig_t;

typedef struct {
  float V[3];
  float invW[3][3];
  uint8_t cal_good;
} full_adv_cal_t;

#endif // types_h

