// LED task API
#pragma once

/** @brief Start the LED animation task (boot sequence + steady run color). */
void ledStartTask();

// Optional runtime LED debug modes
enum LedMode : uint8_t
{
  LED_MODE_STATUS = 0,  // red/orange/green/yellow fault (default)
  LED_MODE_SENSORS = 1, // cycle through sensors with per-sensor colors
  LED_MODE_TILT = 2,    // visualize tilt (hue) and magnitude (brightness)
};

/** @brief Change LED debug mode at runtime. */
void ledSetMode(uint8_t mode);
