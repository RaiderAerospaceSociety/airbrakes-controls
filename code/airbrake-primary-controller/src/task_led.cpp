// LED animation task
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <UMS3.h>

#include "app_config.h"
#include "board.h"

static void task_led(void *param) {
  int color = 0;
  for (;;) {
    ums3.setPixelColor(UMS3::colorWheel(color++));
    vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
  }
}

void led_start_task() {
  xTaskCreatePinnedToCore(task_led, "led", TASK_STACK_LED, nullptr, TASK_PRIO_LED, nullptr, APP_CPU_NUM);
}

