// ===== Bus Setup & Utilities =====
// Brief: Initializes I2C/SPI buses and provides an I2C scan helper.
//* -- Includes --
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "bus.h"
#include "logging.h"
#if SD_PROBE_ON_BOOT
#include <SD.h>
#endif

SemaphoreHandle_t g_spi_mutex = nullptr;
SemaphoreHandle_t g_i2c_mutex = nullptr;
SemaphoreHandle_t g_setup_mutex = nullptr;

//* -- API --
void bus_scan_i2c()
{
#if DEBUG_ENABLED
  LOGF("I2C scan on SDA:%d SCL:%d (clk=%lu Hz)\n", PIN_SDA1, PIN_SCL1, (unsigned long)Wire.getClock());
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    uint8_t err;
    if (g_i2c_mutex)
      xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    Wire.beginTransmission(addr);
    err = Wire.endTransmission(true);
    if (g_i2c_mutex)
      xSemaphoreGive(g_i2c_mutex);
    if (err == 0)
    {
      LOGF(" - 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0)
  {
    LOGLN("I2C scan: no devices found");
  }
  else
  {
    LOGF("I2C scan: %u device(s)\n", found);
  }
#else
  (void)0; // no-op when DEBUG is disabled
#endif
}

void bus_probe_sd()
{
#if SD_PROBE_ON_BOOT
  DEBUGLN("SD: probing...");
  DEBUGF("SD: pins CS=%d SCK=%d MISO=%d MOSI=%d\n", PIN_CS_SD1, PIN_SCK1, PIN_MISO1, PIN_MOSI1);

  // Ensure CS lines are high (avoid contention)
  pinMode(PIN_CS_SD1, OUTPUT);
  digitalWrite(PIN_CS_SD1, HIGH);
  pinMode(PIN_CS_BMP1, OUTPUT);
  digitalWrite(PIN_CS_BMP1, HIGH);

  if (g_spi_mutex)
    xSemaphoreTake(g_spi_mutex, portMAX_DELAY);

  // Try a few SPI clock rates for robustness
  const uint32_t freqs[] = {40000000UL, 20000000UL, 10000000UL, 4000000UL, 1000000UL};
  bool sd_ok = false;
  uint32_t used_hz = 0;

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i)
  {
    uint32_t hz = freqs[i];
    // Re-init default SPI pins in case something altered bus state
    SPI.end();
    SPI.begin(PIN_SCK1, PIN_MISO1, PIN_MOSI1, PIN_CS_SD1);
    if (SD.begin(PIN_CS_SD1, SPI, hz))
    {
      sd_ok = true;
      used_hz = hz;
      break;
    }
  }

  if (g_spi_mutex)
    xSemaphoreGive(g_spi_mutex);

  if (sd_ok)
  {
    DEBUGF("SD: mount OK @ %lu Hz\n", (unsigned long)used_hz);

#ifdef ARDUINO_ARCH_ESP32
    uint8_t ctype = SD.cardType();
    const char *cstr = (ctype == 0 ? "NONE" : (ctype == 1 ? "MMC" : (ctype == 2 ? "SDSC" : (ctype == 3 ? "SDHC/SDXC" : "UNKNOWN"))));
    DEBUGF("SD: cardType=%s (%u)\n", cstr, (unsigned)ctype);
    uint64_t csize = SD.cardSize();
    if (csize > 0)
      DEBUGF("SD: cardSize=%llu MB\n", (unsigned long long)(csize / (1024ULL * 1024ULL)));
#endif

    // Attempt a simple filesystem op and list a few entries
    File root = SD.open("/");
    if (root)
    {
      DEBUGLN("SD: root opened; listing (max 10 entries):");
      int count = 0;
      for (;;)
      {
        File f = root.openNextFile();
        if (!f)
          break;
        if (f.isDirectory())
          DEBUGF("  <DIR> %s\n", f.name());
        else
          DEBUGF("  %8llu  %s\n", (unsigned long long)f.size(), f.name());
        f.close();
        if (++count >= 10)
        {
          DEBUGLN("  ...");
          break;
        }
      }
      root.close();
    }
    else
    {
      DEBUGLN("SD: mount OK but root open failed");
    }

#if SD_PROBE_WRITE_TEST
    // Write a simple test file and read it back
    DEBUGLN("SD: write test -> /test.txt");
    File wf = SD.open("/test.txt", FILE_WRITE);
    if (wf)
    {
      wf.println("testing 1,2,3");
      wf.println("hello bff!");
      wf.close();
      DEBUGLN("SD: write OK");
      File rf = SD.open("/test.txt", FILE_READ);
      if (rf)
      {
        DEBUGLN("SD: read /test.txt ->");
        while (rf.available())
          Serial.write(rf.read());
        rf.close();
      }
      else
      {
        DEBUGLN("SD: open for read failed");
      }
    }
    else
    {
      DEBUGLN("SD: open for write failed");
    }
#endif
  }
  else
  {
    DEBUGLN("SD: probe failed (check CS wiring, power, and SPI pins)");
    DEBUGLN("    Tips: verify 3V3 & GND, short wires, CS unique, and try another card");
  }
#else
  (void)0; // no-op when disabled
#endif
}
