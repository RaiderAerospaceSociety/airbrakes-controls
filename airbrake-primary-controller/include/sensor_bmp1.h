// BMP390 sensor task API
#pragma once

#include <Arduino.h>

/**
 * @brief BMP390 barometer/thermometer snapshot.
 * @note Units and frame:
 *  - temperature_c: degrees Celsius
 *  - pressure_pa: Pascals
 *  - altitude_m: meters above MSL (Earth frame)
 */
typedef struct
{
  double temperature_c; ///< Temperature (Celsius)
  double pressure_pa;   ///< Pressure (Pa)
  double altitude_m;    ///< Altitude (m), Earth frame
  bool valid;           ///< True if the last read succeeded
} bmp_reading_t;

/** @brief Start the BMP390 polling task (SPI). */
void bmp1StartTask();

/** @brief Copy the latest BMP390 reading.
 *  @param out Filled with the latest reading (see units above).
 *  @return true if data is valid.
 */
bool bmp1Get(bmp_reading_t &out);
