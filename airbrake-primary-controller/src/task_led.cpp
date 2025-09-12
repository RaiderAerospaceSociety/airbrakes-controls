// ===== LED Task =====
// Brief: Status LED — red on start, orange while waiting for boot readiness,
// green when fused values are ready, flashing yellow on boot faults.
//* -- Includes --
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <UMS3.h>

#include "app_config.h"
#include "board.h"
#include "services/fc.h"
#include "telemetry.h"
#include "task_led.h"

static volatile uint8_t s_led_mode = LED_MODE_DEFAULT;

void ledSetMode(uint8_t mode) { s_led_mode = mode; }

static uint32_t colorFromHSV(float h_deg, float s, float v)
{
  // Simple HSV->RGB for NeoPixel (s,v in 0..1)
  while (h_deg < 0)
    h_deg += 360.0f;
  while (h_deg >= 360.0f)
    h_deg -= 360.0f;
  float c = v * s;
  float x = c * (1 - fabsf(fmodf(h_deg / 60.0f, 2) - 1));
  float m = v - c;
  float r = 0, g = 0, b = 0;
  if (h_deg < 60)
  {
    r = c;
    g = x;
    b = 0;
  }
  else if (h_deg < 120)
  {
    r = x;
    g = c;
    b = 0;
  }
  else if (h_deg < 180)
  {
    r = 0;
    g = c;
    b = x;
  }
  else if (h_deg < 240)
  {
    r = 0;
    g = x;
    b = c;
  }
  else if (h_deg < 300)
  {
    r = x;
    g = 0;
    b = c;
  }
  else
  {
    r = c;
    g = 0;
    b = x;
  }
  uint8_t R = (uint8_t)(255.0f * (r + m));
  uint8_t G = (uint8_t)(255.0f * (g + m));
  uint8_t B = (uint8_t)(255.0f * (b + m));
  return ((uint32_t)R << 16) | ((uint32_t)G << 8) | (uint32_t)B;
}

static void task_led(void *param)
{
  // Start: solid red
  ums3.setPixelColor(0xFF0000);

  bool blink_on = false;
  uint32_t last_blink_ms = 0;
  const uint32_t blink_period_ms = 400; // ~2.5 Hz
  uint32_t phase_ms = 0;                // for sensor cycling

  for (;;)
  {
    // Gather status snapshots
    svc::FcStatus st;
    bool have_fc = svc::fcGetStatus(st);
    TelemetryRecord rec;
    telemetryGetLatest(rec);
    const auto &fu = rec.fused;
    bool have_fused = true; // fused snapshot present in telemetry

    // Determine phases/faults
    bool sensors_ok = false;
    bool agl_ready = false;
    bool fault = false;

    if (have_fc)
    {
      uint32_t ff = st.flags;
      // Consider IMU1 and BMP1 as required for boot; IMU2 optional
      sensors_ok = (ff & svc::FCF_SENS_IMU1_OK) && (ff & svc::FCF_SENS_BMP1_OK);
      fault = !sensors_ok; // treat missing required sensors as boot fault
    }
    agl_ready = fu.agl_ready;

    uint32_t now = millis();
    if ((now - last_blink_ms) >= blink_period_ms)
    {
      blink_on = !blink_on;
      last_blink_ms = now;
    }

    uint32_t color = 0xFF0000; // default
    switch (s_led_mode)
    {
    case LED_MODE_STATUS:
    {
      if (fault)
      {
        color = blink_on ? 0xFFFF00 : 0x000000; // flashing yellow
      }
      else if (sensors_ok && !agl_ready)
      {
        color = 0xFFA500; // orange
      }
      else if (sensors_ok && agl_ready)
      {
        color = 0x00FF00; // green
      }
      else
      {
        color = 0xFF0000; // red
      }
      break;
    }
    case LED_MODE_SENSORS:
    {
      // Cycle: BMP1 -> IMU1 -> IMU2 (every ~700ms)
      const uint32_t slot_ms = 700;
      phase_ms += LED_PERIOD_MS;
      uint32_t slot = (phase_ms / slot_ms) % 3;
      uint32_t ff = have_fc ? st.flags : 0;
      bool ok = false;
      uint32_t base = 0x000000;
      if (slot == 0)
      {
        base = 0x00FF00;
        ok = (ff & svc::FCF_SENS_BMP1_OK);
      } // green BMP1
      else if (slot == 1)
      {
        base = 0x00FFFF;
        ok = (ff & svc::FCF_SENS_IMU1_OK);
      } // cyan IMU1
      else
      {
        base = 0xFF00FF;
        ok = (ff & svc::FCF_SENS_IMU2_OK);
      } // magenta IMU2
      color = ok ? base : (blink_on ? base : 0x000000);
      break;
    }
    case LED_MODE_TILT:
    {
      // Map tilt azimuth to hue, tilt magnitude to brightness
      float hue = fu.tilt_az_deg360;
      float mag = fu.tilt_deg; // 0..180 deg
      float v = fminf(1.0f, mag / 30.0f);                               // saturate at 30 deg
      float s = sensors_ok ? 1.0f : 0.2f;
      color = colorFromHSV(hue, s, v);
      // If fault, overlay blink to yellow off
      if (fault && !blink_on)
        color = 0x000000;
      break;
    }
    default:
      break;
    }

    ums3.setPixelColor(color);

#if LED_BLUE_HEARTBEAT
    // Blue LED heartbeat patterns for ground debugging
    bool blue_on = false;
    if (fault)
    {
      // Fast blink at ~4 Hz: 100ms on per 250ms cycle
      uint32_t t = now % 250;
      blue_on = (t < 100);
    }
    else if (!sensors_ok)
    {
      // Slow single blip every 2s to show liveness
      uint32_t t = now % 2000;
      blue_on = (t < 80);
    }
    else if (sensors_ok && !agl_ready)
    {
      // Medium 1 Hz blip while waiting for fused readiness
      uint32_t t = now % 1000;
      blue_on = (t < 100);
    }
    else
    {
      // Ready: double‑blip every 2s
      uint32_t t = now % 2000;
      blue_on = (t < 60) || (t >= 300 && t < 360);
    }
    ums3.setBlueLED(blue_on);
#endif
    vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
  }
}

//* -- API --
void ledStartTask()
{
  xTaskCreatePinnedToCore(task_led, "led", TASK_STACK_LED, nullptr, TASK_PRIO_LED, nullptr, APP_CPU_NUM);
}
