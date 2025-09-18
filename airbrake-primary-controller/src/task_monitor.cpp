// ===== Monitoring Task =====
// Brief: Emits either Visualizer (key:value) or Human (fixed-width) lines.
// Refs: docs/monitoring.md, docs/signals.md
//* -- Includes --
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "app_config.h"
#include "logging.h"
#include "sensor_bmp1.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "telemetry.h"
#include "actuator_servo.h"
#include "services/fc.h"
#include "config/actuators_config.h"
#include "services/fusion.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_system.h>
#endif
// !SECTION

//* -- Task --

static void task_monitor(void *param)
{
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    // Handle inbound control commands on Serial (newline terminated)
#if SERIAL_DATA_ENABLE
    if (Serial.available())
    {
      static char cbuf[96];
      static size_t clen = 0;
      while (Serial.available())
      {
        int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\n' || ch == '\r')
        {
          cbuf[clen] = '\0';
          clen = 0;
          if (cbuf[0] == '!' && strncmp(cbuf, "!cmd:", 5) == 0)
          {
            const char *cmd = cbuf + 5;
            if (strcasecmp(cmd, "soft_reset") == 0)
            {
              svc::fusionSoftReset();
              svc::fcSoftReset();
              Serial.println(">evt:soft_reset");
            }
            else if (strcasecmp(cmd, "hard_reset") == 0)
            {
#if defined(ARDUINO_ARCH_ESP32)
              Serial.println(">evt:hard_reset");
              delay(50);
              esp_restart();
#endif
            }
          }
        }
        else if (clen < sizeof(cbuf) - 1)
        {
          cbuf[clen++] = (char)ch;
        }
      }
    }
#endif
    // Fetch data used for monitoring lines
#if SERIAL_DATA_ENABLE
    TelemetryRecord rec;
    telemetryGetLatest(rec);
    const auto &fu = rec.fused;

    // Mode 0: Visualizer (key:value)
#if (MON_MODE == 0)
    auto kv_f = [](const char *key, float val, int prec)
    {
      Serial.print(", "); Serial.print(key); Serial.print(":");
      if (isnan(val)) Serial.print("nan"); else Serial.print(val, prec);
    };
    auto kv_i = [](const char *key, int32_t val)
    {
      Serial.print(", "); Serial.print(key); Serial.print(":"); Serial.print(val);
    };
    auto kv_s = [](const char *key, const char *s)
    {
      Serial.print(", "); Serial.print(key); Serial.print(":"); Serial.print(s);
    };
    if (MON_INCLUDE_TS) { Serial.print("ts_ms:"); Serial.print((uint32_t)millis()); }
    // Battery (1S Li‑ion 3.0..4.2V typical scale)
    kv_f("vbat_v", rec.sys.vbat_mv / 1000.0f, 3);
    // Bus error counters
    kv_i("i2c_errs", rec.sys.i2c_errs);
    kv_i("spi_errs", rec.sys.spi_errs);
    // FC state (string + code)
    auto state_name = [](uint8_t s) -> const char *
    {
      switch (s) {
        case svc::FC_SAFE: return "SAFE";
        case svc::FC_PREFLIGHT: return "PREFLIGHT";
        case svc::FC_ARMED_WAIT: return "ARMED_WAIT";
        case svc::FC_BOOST: return "BOOST";
        case svc::FC_POST_BURN_HOLD: return "POST_HOLD";
        case svc::FC_WINDOW: return "WINDOW";
        case svc::FC_DEPLOYED: return "DEPLOYED";
        case svc::FC_RETRACTING: return "RETRACT";
        case svc::FC_LOCKED: return "LOCKED";
        case svc::FC_ABORT_LOCKOUT: return "ABORT_LOCKOUT";
        default: return "UNKNOWN";
      }
    };
    kv_s("fc_state_str", state_name(rec.sys.fc_state));
    kv_i("fc_state", (int32_t)rec.sys.fc_state);
    kv_i("fc_flags", (int32_t)rec.sys.fc_flags);
    // Status lights (booleans)
    kv_i("sens_imu1_ok", rec.sys.sens_imu1_ok);
    kv_i("sens_bmp1_ok", rec.sys.sens_bmp1_ok);
    kv_i("sens_imu2_ok", rec.sys.sens_imu2_ok);
    kv_i("baro_agree", rec.sys.baro_agree);
    kv_i("mach_ok", rec.sys.mach_ok);
    kv_i("tilt_ok", rec.sys.tilt_ok);
    kv_i("tilt_latch", rec.sys.tilt_latch);
    kv_i("liftoff_det", rec.sys.liftoff_det);
    kv_i("burnout_det", rec.sys.burnout_det);
    kv_i("lockout", rec.sys.fc_state == svc::FC_ABORT_LOCKOUT ? 1 : 0);
    // Times (s)
    kv_f("t_since_launch_s", rec.sys.fc_t_since_launch_s, 2);
    kv_f("t_to_apogee_s", rec.sys.fc_t_to_apogee_s, 2);
    // Airbrake cmd/actual
    kv_f("cmd_deg", (float)rec.ctl.airbrake_cmd_deg, 2);
    kv_f("act_deg", (float)rec.ctl.airbrake_actual_deg, 2);
    // Fused core for graphs/gauges
    kv_i("agl_ready", fu.agl_ready);
    kv_f("temp_c", fu.temp_c, 2);
    kv_f("agl_fused_m", fu.agl_fused_m, 3);
    kv_f("vz_fused_mps", fu.vz_fused_mps, 3);
    kv_f("az_imu1_mps2", fu.az_imu1_mps2, 3);
    kv_f("tilt_deg", fu.tilt_deg, 2);
    kv_f("tilt_az_deg360", fu.tilt_az_deg360, 1);
    kv_f("mach_cons", fu.mach_cons, 4);
    // Servo status (if enabled)
#if SERVO_ENABLE
    extern ServoStatus servoGetStatus();
    ServoStatus sv = servoGetStatus();
    kv_i("servo_open", sv.open ? 1 : 0);
    kv_i("servo_cmd_us", (int32_t)sv.cmd_us);
    kv_i("servo_min_us", (int32_t)sv.min_us);
    kv_i("servo_max_us", (int32_t)sv.max_us);
#endif
    // Optional fusion sub-values for verification
#if MON_SHOW_FUSION_PARTS
    kv_f("agl_fused_m", fu.agl_fused_m, 3);
    kv_f("agl_bmp1_m", fu.agl_bmp1_m, 3);
    kv_f("agl_imu1_m", fu.agl_imu1_m, 3);
    kv_f("vz_fused_mps", fu.vz_fused_mps, 3);
    kv_f("vz_baro_mps", fu.vz_mps, 3);
#if FUSION_USE_ACC_INT
    kv_f("vz_acc_mps", fu.vz_acc_mps, 3);
#endif
#endif // MON_SHOW_FUSION_PARTS
    Serial.println();
#elif (MON_MODE == 1)
    // Mode 1: Human (fixed-width, signed-aware)
    svc::FcStatus st; svc::fcGetStatus(st);
    auto state_name = [](uint8_t s) -> const char *
    {
      switch (s) {
        case svc::FC_SAFE: return "SAFE";
        case svc::FC_PREFLIGHT: return "PREFLIGHT";
        case svc::FC_ARMED_WAIT: return "ARMED_WAIT";
        case svc::FC_BOOST: return "BOOST";
        case svc::FC_POST_BURN_HOLD: return "POST_HOLD";
        case svc::FC_WINDOW: return "WINDOW";
        case svc::FC_DEPLOYED: return "DEPLOYED";
        case svc::FC_RETRACTING: return "RETRACT";
        case svc::FC_LOCKED: return "LOCKED";
        case svc::FC_ABORT_LOCKOUT: return "ABORT";
        default: return "UNKNOWN";
      }
    };
    uint32_t ff = rec.sys.fc_flags;
    int mach_ok = (ff & svc::FCF_MACH_OK) ? 1 : 0;
    int tilt_ok = (ff & svc::FCF_TILT_OK) ? 1 : 0;
    int tilt_lock = (ff & svc::FCF_TILT_LATCH) ? 1 : 0;
    if (MON_INCLUDE_TS) Serial.printf("%08lu ", (uint32_t)millis());
    Serial.printf("%-10s ", state_name(rec.sys.fc_state));
    Serial.printf("M:%d T:%d L:%d ", mach_ok, tilt_ok, tilt_lock);
    Serial.printf("cmd:%+05.1f ", (float)rec.ctl.airbrake_cmd_deg);
    Serial.printf("tilt:%+06.2f ", fu.tilt_deg);
    Serial.printf("mach:%0.3f ", fu.mach_cons);
    Serial.printf("vz:%+07.2f ", fu.vz_fused_mps);
    Serial.printf("agl:%+07.2f", fu.agl_fused_m);
    Serial.println();
#endif // MON_MODE
#endif // SERIAL_DATA_ENABLE

    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOGGER_PERIOD_MS));
  }
}

// !SECTION

void monitorStartTask()
{
  xTaskCreatePinnedToCore(task_monitor, "monitor", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
