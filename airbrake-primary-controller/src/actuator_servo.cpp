#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "rtos_mutex.h"
#include "actuator_servo.h"
#include "config/actuators_config.h"
#include "telemetry.h"
#include "services/fc.h"

#if SERVO_ENABLE

#if !defined(ARDUINO_ARCH_ESP32)
#error "Servo actuator currently implemented for ESP32-class (Feather S3) only"
#endif

static uint16_t s_min_us = SERVO_MIN_US;
static uint16_t s_max_us = SERVO_MAX_US;
static uint16_t s_cmd_us = (SERVO_MIN_US + SERVO_MAX_US) / 2;
static bool s_open = false;
static uint8_t s_pin = SERVO_PWM_PIN;

static inline uint32_t us_to_duty(uint16_t us)
{
  // LEDC duty counts: (us / period_us) * ((1<<res)-1)
  const uint32_t period_us = 1000000UL / SERVO_PWM_FREQ_HZ; // 20000us at 50Hz
  const uint32_t maxd = (1UL << SERVO_PWM_RES_BITS) - 1UL;
  uint32_t duty = (uint32_t)((((uint64_t)us) * maxd) / period_us);
  if (duty > maxd) duty = maxd;
  return duty;
}

void servoWriteUS(uint16_t us)
{
  if (us < s_min_us) us = s_min_us;
  if (us > s_max_us) us = s_max_us;
  s_cmd_us = us;
  uint32_t duty = us_to_duty(us);
  ledcWrite(SERVO_PWM_CHANNEL, duty);
}

void servoOpen()  { s_open = true;  servoWriteUS(s_max_us); }
void servoClose() { s_open = false; servoWriteUS(s_min_us); }
void servoCenter(){ servoWriteUS((s_min_us + s_max_us)/2); }

ServoStatus servoGetStatus()
{
  ServoStatus st{ s_min_us, s_max_us, s_cmd_us, s_open };
  return st;
}

static void servo_boot_sweep()
{
  ENTER_CRITICAL(g_setup_mutex);
  LOGLN("[servo] Boot sweep: slow");
  for (int us = s_min_us; us <= (int)s_max_us; us += 10) { servoWriteUS(us); delay(15); }
  for (int us = s_max_us; us >= (int)s_min_us; us -= 10) { servoWriteUS(us); delay(15); }
  LOGLN("[servo] Boot sweep: fast");
  for (int us = s_min_us; us <= (int)s_max_us; us += 20) { servoWriteUS(us); delay(5); }
  for (int us = s_max_us; us >= (int)s_min_us; us -= 20) { servoWriteUS(us); delay(5); }
  LOGLN("[servo] Boot sweep: medium");
  for (int us = s_min_us; us <= (int)s_max_us; us += 10) { servoWriteUS(us); delay(10); }
  for (int us = s_max_us; us >= (int)s_min_us; us -= 10) { servoWriteUS(us); delay(10); }
  LOGLN("[servo] Boot sweep: retract");
  servoClose();
  EXIT_CRITICAL(g_setup_mutex);
}

void servoInit()
{
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
  ledcAttachPin(s_pin, SERVO_PWM_CHANNEL);
  servoCenter();
  // Run the boot sweep under setup mutex so messages are visible and exclusive
  servo_boot_sweep();
}

#if SERVO_BENCH

static void task_servo_bench(void *param)
{
  LOGLN("[servo] Bench mode active. Commands: !servo:open|close|center|sweep|us:<µs>|range:<min>:<max>" );
  for(;;)
  {
    if (Serial.available())
    {
      static char buf[96];
      static size_t n=0;
      while (Serial.available())
      {
        int c = Serial.read(); if (c < 0) break;
        if (c=='\n' || c=='\r') { buf[n]='\0'; n=0; if (buf[0]) {
            if (strncmp(buf, "!servo:", 7)==0) {
              const char* cmd = buf+7;
              if (strcmp(cmd, "open")==0) { servoOpen(); Serial.println(">servo:open"); }
              else if (strcmp(cmd, "close")==0) { servoClose(); Serial.println(">servo:close"); }
              else if (strcmp(cmd, "center")==0) { servoCenter(); Serial.println(">servo:center"); }
              else if (strcmp(cmd, "sweep")==0) { servo_boot_sweep(); Serial.println(">servo:sweep_done"); }
              else if (strncmp(cmd, "us:",3)==0) { int us=atoi(cmd+3); servoWriteUS(us); Serial.printf(">servo:us:%d\n", us); }
              else if (strncmp(cmd, "range:",6)==0) { int mn=0,mx=0; if (sscanf(cmd+6, "%d:%d", &mn, &mx)==2) { s_min_us=mn; s_max_us=mx; servoClose(); Serial.printf(">servo:range:%d:%d\n", mn,mx);} }
            }
          }}
        else if (n < sizeof(buf)-1) { buf[n++] = (char)c; }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#else // normal FSM-driven mode

static void task_servo_ctrl(void *param)
{
  uint32_t last_stamp = 0;
  for(;;)
  {
    TelemetryRecord rec{};
    telemetryGetLatest(rec);
    // Watchdog: if telemetry stalls, retract
    if (rec.hdr.timestamp_ms == 0 || rec.hdr.timestamp_ms == last_stamp)
    {
      servoClose();
      vTaskDelay(pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
      continue;
    }
    last_stamp = rec.hdr.timestamp_ms;

    const TelemetrySystem &sys = rec.sys;
    const TelemetryFused &fu = rec.fused;

    bool health_ok = (sys.sens_imu1_ok && sys.sens_bmp1_ok && sys.sens_imu2_ok && fu.agl_ready);
    bool tilt_ok = (sys.tilt_latch == 0);
    bool mach_ok = (!isnan(fu.mach_cons) && fu.mach_cons < 0.5f);
    bool in_window = (sys.fc_state == svc::FC_WINDOW);
    bool post_burn = (sys.fc_state != svc::FC_BOOST);
    bool abort_or_lock = (sys.fc_state == svc::FC_ABORT_LOCKOUT || sys.fc_state == svc::FC_LOCKED);
    bool near_apogee = (!isnan(sys.fc_t_to_apogee_s) && sys.fc_t_to_apogee_s <= 1.0f);

    bool should_open = false;
    if (!abort_or_lock && health_ok && tilt_ok && mach_ok && post_burn && in_window)
      should_open = true;

    if (abort_or_lock || !health_ok || !tilt_ok || near_apogee)
      should_open = false;

    if (should_open != s_open)
    {
      if (should_open) servoOpen(); else servoClose();
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
  }
}

#endif

void servoStartTask()
{
  // Ensure PWM is set up
  servoInit();
#if SERVO_BENCH
  xTaskCreatePinnedToCore(task_servo_bench, "servo_bench", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
#else
  xTaskCreatePinnedToCore(task_servo_ctrl, "servo_ctrl", 3072, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
#endif
}

#endif // SERVO_ENABLE
