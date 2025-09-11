// ===== Pins Map =====
// Brief: Centralized pin map with override-friendly macros.
//* -- Overview --
// Centralized pin map with override-friendly macros.
// You can override any of these via PlatformIO build_flags, e.g.:
//   -D PIN_SDA1=8 -D PIN_SCL1=9 -D PIN_SCK1=36 -D PIN_MISO1=37 -D PIN_MOSI1=35 -D PIN_CS_BMP1=5

#pragma once

// I2C
/** @brief I2C SDA pin number (override via -D PIN_SDA1=...). */
#ifndef PIN_SDA1
#define PIN_SDA1 33 // Feather S3 typical SDA
#endif
/** @brief I2C SCL pin number (override via -D PIN_SCL1=...). */
#ifndef PIN_SCL1
#define PIN_SCL1 38 // Feather S3 typical SCL
#endif

// SPI

#ifndef PIN_SCK1
#define PIN_SCK1 36 // Feather S3 SCK
#endif
#ifndef PIN_MISO1
#define PIN_MISO1 37 // Feather S3 MISO
#endif
#ifndef PIN_MOSI1
#define PIN_MOSI1 35 // Feather S3 MOSI
#endif

// SPI (cs)
#ifndef PIN_CS_BMP1
#define PIN_CS_BMP1 10 // Default CS for BMP1
#endif
#ifndef PIN_CS_SD1
#define PIN_CS_SD1 7 // Default CS for SD1
#endif

// USFSMAX DRDY not used; polling is employed
// !SECTION
