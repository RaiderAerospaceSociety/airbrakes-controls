// ===== Main Application =====
// Brief: Initializes board, buses, and starts all tasks.
// Refs: docs/architecture.md
//* -- Includes --
// Core
#include <Arduino.h>
#include <UMS3.h>

//* -- App Includes --
// App modules
#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "board.h"
#include "sensor_bmp390.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "task_led.h"
#include "task_logger.h"
#include "telemetry.h"
#include "services/fusion.h"
#include "services/fc.h"
#if SD_PROBE_ON_BOOT
#include <SPI.h>
#include <SD.h>
#endif
//

// Note: telemetryStartTasks() declared in telemetry.h

//* -- Globals --
// Note: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
//

//* -- Setup --
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  // Init logging (mutex) and shared buses
  logging_setup_mutex();
  bus_setup();
  delay(200);
  bus_scan_i2c();

  // Board setup
  ums3.begin();
  ums3.setPixelBrightness(255 / 3);
  ums3.setPixelPower(true);
  delay(50);

  // Set initial LED to red; task_led will update as subsystems come online
  ums3.setPixelColor(0xFF0000);

#if SD_PROBE_ON_BOOT
  // Quick SD wiring probe (SPI mode) using PIN_CS_SD1
  {
    DEBUGLN("SD: probing...");
    DEBUGF("SD: pins CS=%d SCK=%d MISO=%d MOSI=%d\n", PIN_CS_SD1, PIN_SCK1, PIN_MISO1, PIN_MOSI1);
    pinMode(PIN_CS_SD1, OUTPUT);
    digitalWrite(PIN_CS_SD1, HIGH); // deselect SD by default
    // Ensure other SPI CS lines are high (avoid contention)
    pinMode(PIN_CS_BMP1, OUTPUT);
    digitalWrite(PIN_CS_BMP1, HIGH);
    if (g_spi_mutex) xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
    // Try a few SPI clock rates for robustness
    const uint32_t freqs[] = { 40000000UL, 20000000UL, 10000000UL, 4000000UL, 1000000UL };
    bool sd_ok = false; uint32_t used_hz = 0;
    for (size_t i = 0; i < sizeof(freqs)/sizeof(freqs[0]); ++i) {
      uint32_t hz = freqs[i];
      // Re-init default SPI pins in case something altered bus state
      SPI.end(); SPI.begin(PIN_SCK1, PIN_MISO1, PIN_MOSI1, PIN_CS_SD1);
      if (SD.begin(PIN_CS_SD1, SPI, hz)) { sd_ok = true; used_hz = hz; break; }
    }
    if (g_spi_mutex) xSemaphoreGive(g_spi_mutex);
    if (sd_ok) {
      // Bus/pin info
      DEBUGF("SD: mount OK @ %lu Hz\n", (unsigned long)used_hz);
      // Card type/size (if available in this core)
      #ifdef ARDUINO_ARCH_ESP32
      uint8_t ctype = SD.cardType();
      const char* cstr = (ctype == 0 ? "NONE" : (ctype == 1 ? "MMC" : (ctype == 2 ? "SDSC" : (ctype == 3 ? "SDHC/SDXC" : "UNKNOWN"))));
      DEBUGF("SD: cardType=%s (%u)\n", cstr, (unsigned)ctype);
      uint64_t csize = SD.cardSize();
      if (csize > 0) DEBUGF("SD: cardSize=%llu MB\n", (unsigned long long)(csize / (1024ULL*1024ULL)));
      #endif
      // Attempt a simple filesystem op and list a few entries
      File root = SD.open("/");
      if (root) {
        DEBUGLN("SD: root opened; listing (max 10 entries):");
        int count = 0;
        for (;;) {
          File f = root.openNextFile();
          if (!f) break;
          if (f.isDirectory()) DEBUGF("  <DIR> %s\n", f.name());
          else DEBUGF("  %8llu  %s\n", (unsigned long long)f.size(), f.name());
          f.close();
          if (++count >= 10) { DEBUGLN("  ..."); break; }
        }
        root.close();
      } else {
        DEBUGLN("SD: mount OK but root open failed");
      }

#if SD_PROBE_WRITE_TEST
      // Write a simple test file and read it back
      DEBUGLN("SD: write test -> /test.txt");
      File wf = SD.open("/test.txt", FILE_WRITE);
      if (wf) {
        wf.println("testing 1,2,3");
        wf.println("hello bff!");
        wf.close();
        DEBUGLN("SD: write OK");
        File rf = SD.open("/test.txt", FILE_READ);
        if (rf) {
          DEBUGLN("SD: read /test.txt ->");
          while (rf.available()) Serial.write(rf.read());
          rf.close();
        } else {
          DEBUGLN("SD: open for read failed");
        }
      } else {
        DEBUGLN("SD: open for write failed");
      }
#endif
    } else {
      DEBUGLN("SD: probe failed (check CS wiring, power, and SPI pins)");
      DEBUGLN("    Tips: verify 3V3 & GND, short wires, CS unique, and try another card");
    }
  }
#endif

  // Start tasks
  telemetryStartTasks();
  bmp390StartTask();
  imu1StartTask();
  imu2StartTask();
  svc::fusionStartTask();
  svc::fcStartTask();
  ledStartTask();
  loggerStartTask();
}
//

//* -- Loop --
void loop() {
  vTaskDelay(portMAX_DELAY);
}
//
